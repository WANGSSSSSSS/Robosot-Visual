//
// Created by wang_shuai on 19-9-26.
//
#include "../include/RobsotRaceVisual/ColorSupport.h"

cv::Mat BasicColor::morph_process(cv::Mat binary_mat)
{
    cv::Mat _morph_mat(binary_mat.size(),CV_8UC1,cv::Scalar::all(0));

    cv::Mat erode_kernel1=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(erode_kernel_size1*2+1,erode_kernel_size1*2+1));

    cv::Mat dilate_kernel1=cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(dilate_kernel_size1*2+1,dilate_kernel_size1*2+1));

    cv::morphologyEx(binary_mat,_morph_mat,cv::MORPH_ERODE,erode_kernel1);

    cv::morphologyEx(_morph_mat,_morph_mat,cv::MORPH_DILATE,dilate_kernel1);

    cv::GaussianBlur(_morph_mat,_morph_mat,cv::Size(gua_kernal_size*2+1,gua_kernal_size*2+1),0,0);

    return _morph_mat;
}

cv::Mat BasicColor::bgr_process(cv::Mat src)
{
    cv::Mat binary_mat;

    cv::inRange(src,
                cv::Scalar(rgb_b_min,rgb_g_min,rgb_r_min),
                cv::Scalar(rgb_b_max,rgb_g_max,rgb_r_max),
                binary_mat);

    return binary_mat;
}

cv::Mat BasicColor::hsv_process(cv::Mat src)
{
    cv::Mat binary_mat;

    cv::Mat hsv_mat;

    cv::cvtColor(src,hsv_mat,cv::COLOR_BGR2HSV);

    cv::inRange(hsv_mat,
                cv::Scalar(hsv_h_min,hsv_s_min,hsv_v_min),
                cv::Scalar(hsv_h_max,hsv_s_max,hsv_v_max),
                binary_mat);

    return binary_mat;
}
// 找到最小包含色块的矩形
Rects BasicColor::min_Rects(cv::Mat binary_mat)
{
    int threhold1 = 100;

    int threhold2 = 200;

    cv::Mat canny_mat;

    cv::Canny(binary_mat,canny_mat,threhold1,threhold2);

    std::vector<std::vector<cv::Point> > contours;

    std::vector<cv::Rect> rects;

    cv::findContours(canny_mat,contours,cv::RETR_EXTERNAL,cv::CHAIN_APPROX_NONE);

    for(int i=0;i<contours.size();i++)
    {
        rects.push_back(cv::boundingRect(contours[i]));
    }

    return rects;
}

// 判断当前矩形是否合理并对矩形进行扩展
// 参数表： 矩形 图像宽 图像长 二值图
Rects BasicColor::final_Rects(Rects rects, int cols, int rows, cv::Mat binary_mat)
{
    for(int i=0;i<rects.size();i++)
    {
        if(rects[i].height<40||rects[i].width<40||fabsf(rects[i].height*1.0f/float(rects[i].width))>5.0||fabsf((rects[i].height)*1.0f/float(rects[i].width))<0.2)
        {
            rects.erase(rects.begin()+i);

            i--;

            continue;
        }
        //borden the bounding box
        rects[i].height +=(6);

        rects[i].width +=(6);

        rects[i].x-=3;

        rects[i].y-=3;

        while(rects[i].x<=3)rects[i].x+=1;

        while(rects[i].y<=3)rects[i].y+=1;

        while(rects[i].br().x>=cols-3)rects[i].width--;

        while(rects[i].br().y>=rows-3)rects[i].height--;

        cv::Mat random_x(1,100,CV_32FC1);

        cv::Mat random_y(1,100,CV_32FC1);

        cv::randu(random_x,cv::Scalar(float(rects[i].tl().x+3.0f)),cv::Scalar(float(rects[i].br().x-3.0f)));

        cv::randu(random_y,cv::Scalar(float(rects[i].tl().y+3.0f)),cv::Scalar(float(rects[i].br().y-3.0f)));

        // 采样
        int count = 0;

        for(int k=0; k<100; k++)
        {
            float x_ = random_x.at<float>(0,k);

            float y_ = random_y.at<float>(0,k);

            char  element = binary_mat.at<char>(int(y_),int(x_));

            if(element == 0)count++;
        }
        if(count>40)
        {
            rects.erase(rects.begin()+i);

            i--;
        }
    }
    return rects;
}

//写入文件

void BasicColor::write()
{
    std::string path = WRITER_PATH+color_name+ ".yaml";
    YAML::Node yamlconfig = YAML::LoadFile(path);

    yamlconfig["rgb_r_min"] = rgb_r_min;

    yamlconfig["rgb_r_max"] = rgb_r_max;

    yamlconfig["rgb_g_min"] = rgb_g_min;

    yamlconfig["rgb_g_max"] = rgb_g_max;

    yamlconfig["rgb_b_min"] = rgb_b_min;

    yamlconfig["rgb_b_max"] = rgb_b_max;

    yamlconfig["hsv_h_min"] = hsv_h_min;

    yamlconfig["hsv_h_max"] = hsv_h_max;

    yamlconfig["hsv_s_min"] = hsv_s_min;

    yamlconfig["hsv_s_max"] = hsv_s_max;

    yamlconfig["hsv_v_min"] = hsv_v_min;

    yamlconfig["hsv_v_max"] = hsv_v_max;

    yamlconfig["erode_kernel_size1"] = erode_kernel_size1;

    yamlconfig["dilate_kernel_size1"] = dilate_kernel_size1;

    yamlconfig["gua_kernal_size"] = gua_kernal_size;
    //yamlconfig["button"] = config.button;
    //yamlconfig["catch_num"] = config.catch_num;

    std::ofstream file;

    file.open(path.c_str());

    if (!file) 
    {
        ROS_INFO("file write error");

    }
    else 
    {
        ROS_INFO("write");
    }

    file.flush();

    file << yamlconfig;

    file.close();
};

//RosColor 主流程

PointColors RosColor::color_main(cv::Mat src)
{
    PointColors pointColors;

    pointColors.clear();

    if(src.empty() == 1)
    {
        return pointColors;
    }

    cv::Mat binary_mat = hsv_process(src) & bgr_process(src);

    cv::Mat morph_mat = morph_process(binary_mat);

    get_alpha(binary_mat);

    get_ball(binary_mat);

    Rects rects = min_Rects(morph_mat);

    Rects final_rects = final_Rects(rects,binary_mat.cols,binary_mat.rows,binary_mat);

    cv::Mat temp_mat;


    unsigned long length = final_rects.size();

    for(unsigned long i = 0; i < length ;i++)
    {
        temp_mat = cv::Mat::zeros(binary_mat.size(),CV_8UC1);

        cv::Mat temp1 = binary_mat(final_rects[i]);

        cv::Mat temp2 = temp_mat(final_rects[i]);

        temp1.copyTo(temp2);

        cv::Mat edge;

        cv::Canny(temp_mat,edge,100,200);

        cv::blur(edge,edge,cv::Size(3,3));

        contours _contours;

        cv::findContours(edge,_contours,cv::RETR_LIST,cv::CHAIN_APPROX_NONE);

        unsigned long  max = 0;

        unsigned long index=0;

        for(unsigned long i=0;i<_contours.size();i++)
        {
            if(max<_contours[i].size())
            {
                index=i;

                max = _contours[i].size();
            }
        }

        cv::RotatedRect rotatedRect = cv::fitEllipse(_contours[i]);

        PointColor pointColor;

        pointColor.color_name = color_name;

        pointColor.Point = rotatedRect.center;

        pointColors.push_back(pointColor);

    }

    return pointColors;

}






RosColor::RosColor(std::string color_name):_nh("/color_select/"+color_name),paramServer(_nh)
{
    this->color_name = color_name;

    paramServer.setCallback(boost::bind(&RosColor::dy_function,this,_1,_2));
}

float RosColor::Alpha()
{
    return alpha;
}

void RosColor::get_alpha(cv::Mat binary_mat)
{
    unsigned long rows = binary_mat.rows;

    unsigned long cols = binary_mat.cols;//no problem ,i thought

    unsigned long mid = cols>>1;

    float white_sum = 0;

    float alpha_ = 0;

    for(size_t i = 0; i< rows; i++)
    {
        for(size_t j = 0;j< cols; j++)
        {
            if(binary_mat.at<uchar>(i,j) != 0)
            {
                white_sum ++;

                alpha_ += (i-mid);
            }
        }
    }

    if(white_sum > 0) // 用来pid控制的角度信息
    {
        alpha = alpha_/white_sum;
    }
    else //没有看见球
    {
        alpha = -9999.f;
    }
}

void RosColor::get_ball(cv::Mat binart_mat)
{
    // judge whether the ball was cathed

    // TODO
}


void RosColor::update_bgr_param()
{
    _nh.getParam("/color_select/"+color_name+"/rgb_r_min",rgb_r_min);

    _nh.getParam("/color_select/"+color_name+"/rgb_r_max",rgb_r_max);

    _nh.getParam("/color_select/"+color_name+"/rgb_g_min",rgb_g_min);

    _nh.getParam("/color_select/"+color_name+"/rgb_g_max",rgb_g_max);

    _nh.getParam("/color_select/"+color_name+"/rgb_b_min",rgb_b_min);

    _nh.getParam("/color_select/"+color_name+"/rgb_b_max",rgb_b_max);
}


void RosColor::update_hsv_param()
{
    _nh.getParam("/color_select/"+color_name+"/hsv_h_min",hsv_h_min);

    _nh.getParam("/color_select/"+color_name+"/hsv_h_max",hsv_h_max);

    _nh.getParam("/color_select/"+color_name+"/hsv_s_min",hsv_s_min);

    _nh.getParam("/color_select/"+color_name+"/hsv_s_max",hsv_s_max);

    _nh.getParam("/color_select/"+color_name+"/hsv_v_min",hsv_v_min);

    _nh.getParam("/color_select/"+color_name+"/hsv_v_max",hsv_v_max);
}


void RosColor::update_param()
{
    update_hsv_param();

    update_bgr_param();

    _nh.getParam("/color_select/"+color_name+"/erode_kernel_size1",erode_kernel_size1);

    _nh.getParam("/color_select/"+color_name+"/gua_kernal_size" ,gua_kernal_size);
}


void RosColor::set_color(std::string color_name)
{
    this->color_name = color_name;
}

void RosColor::dy_function(RobsotRaceVisual::dy_colorConfig &config, int level)
{
    update_param();

    if(level >= 0)write();//第一次初始化level为-1
}


