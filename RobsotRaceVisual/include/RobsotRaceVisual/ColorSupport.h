//
// Created by wang_shuai on 19-9-26.
//

#ifndef ROBSOTRACEVISUAL_COLORSUPPORT_H
#define ROBSOTRACEVISUAL_COLORSUPPORT_H

#include "GeneralSupport.h"

class BasicColor
{
public:
    std::string color_name;
    int rgb_r_min;
    int rgb_r_max;
    int rgb_g_min;
    int rgb_g_max;
    int rgb_b_min;
    int rgb_b_max;

    int hsv_h_min;
    int hsv_h_max;
    int hsv_s_min;
    int hsv_s_max;
    int hsv_v_min;
    int hsv_v_max;

    int erode_kernel_size1;

    int dilate_kernel_size1;

    int gua_kernal_size;

    BasicColor(){};

    cv::Mat hsv_process(cv::Mat src);

    cv::Mat bgr_process(cv::Mat src);

    cv::Mat morph_process(cv::Mat binary_mat);

    Rects min_Rects(cv::Mat binary_mat);

    Rects final_Rects(Rects rects,int cols,int rows,cv::Mat binary_mat);

    void write();

    virtual PointColors color_main(cv::Mat src) = 0;
};

class RosColor : public BasicColor
{
private:
    ros::NodeHandle _nh;

    dy_colorServer  paramServer;

    bool ball = 0;// judge whether catch the ball

    float alpha = -99999;//used to pid control

public:
    RosColor(){};

    RosColor(std::string color_name);

    //dy_colorServer::CallbackType f;

    void set_color(std::string color_name);

    void update_hsv_param();

    void update_bgr_param();

    void update_param();

    void dy_function(RobsotRaceVisual::dy_colorConfig &config, int level);

    float Alpha();

    void  get_alpha(cv::Mat binary_mat);

    void  get_ball(cv::Mat binart_mat);

    //virtual contours get_edge(cv::Mat binary_mat) = 0;

    PointColors color_main(cv::Mat src);

    friend bool ros::NodeHandle::getParam(const std::string &key, int&i)const;
};


#endif //ROBSOTRACEVISUAL_COLORSUPPORT_H
