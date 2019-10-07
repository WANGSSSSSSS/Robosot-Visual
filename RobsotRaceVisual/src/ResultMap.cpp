//
// Created by wang_shuai on 19-9-26.
//

#include "../include/RobsotRaceVisual/ResultMap.h"

Map::Map():nh("~"),Green("Green"),Blue("Blue"),Red("Red")
{

    Green.update_param();

    Red.update_param();

    Blue.update_param();

    sub_occupied_grid = nh.subscribe(GRID_SUBSCRIBE_TOPIC,1,&Map::update_occupied_grid,this);

    sub_src = nh.subscribe(SRC_SUBSCRIBE_TOPIC,1,&Map::update_src,this);

    sub_change = nh.subscribe(CHANGE_SUBSCRIBE_TOPIC,1,&Map::update_change,this);

    sub_range_mat = nh.subscribe(DEP_SUBSCRIBE_TOPIC,1,&Map::update_range_mat,this);

    sub_robot_infomation = nh.subscribe(ROB_SUBSCRIBE_TOPIC,1,&Map::update_robot_information,this);

    pub_angle = nh.advertise<std_msgs::Float32>(ANGLE_PUBLISH_TOPIC,1);

    pub_location = nh.advertise<vwbot_controller::PoseAndColor>(LOCAT_PUBLISH_TOPIC,1);

}

void Map::update_range_mat(sensor_msgs::ImageConstPtr msg)
{
    cv_bridge::toCvShare(msg,"16UC1")->image.copyTo(range_mat);
}

void Map::update_src(sensor_msgs::ImageConstPtr msg)
{
    cv_bridge::toCvShare(msg,"bgr8")->image.copyTo(src);
}

void Map::update_change(std_msgs::Bool::ConstPtr msg)
{
    not_change = msg->data;
}

void Map::update_robot_information(geometry_msgs::PoseStamped msg)
{
    robot_x = (float)msg.pose.position.x;

    robot_y = (float)msg.pose.position.y;

    float robot_angle = from_q_to_alpha(msg.pose.orientation.w,msg.pose.orientation.z);

    compute_line(robot_angle , 30);
}

void Map::update_occupied_grid(nav_msgs::OccupancyGrid::ConstPtr msg)
{
    OccupancyGrid = *msg;
}

bool Map::judge_point(float x, float y)
{
    float resolution = OccupancyGrid.info.resolution;

    float x0 = (float)OccupancyGrid.info.origin.position.x;

    float y0 = (float)OccupancyGrid.info.origin.position.y;

    int width = OccupancyGrid.info.width;

    int _x = (int)((x-x0)/resolution);

    int _y = (int)((y-y0)/resolution);

    bool in_field = 1;

    in_field = in_field && (y < bounder_up);

    in_field = in_field && (y > bounder_down);

    in_field = in_field && (x > bounder_left);

    in_field = in_field && (x < bounder_right);

    return (OccupancyGrid.data[width*_y+_x] <= 85) && in_field;
}

float Map::get_depth(float x, float y)
{
    float z = 0;

    float count = 0;

    for(int i=0;i<range_mat.rows;i++)
    {
        short *p = range_mat.ptr<short>(i);

        for(int j=0;j<range_mat.cols;j++)
        {
            if((i-y)*(i-y)+(j-x)*(j-x)<10)
            {
                if(isnanf((float)p[j]))continue;

                if(abs(p[j])>4000)continue;

                z+=(float)p[j];

                count++;
            }
        }
    }

    if(count < 1)
    {
        return -99999.9;
    }
    else
    {
        return z/count;
    }
}

float Map::get_distance(float x, float y, size_t i)
{
    float dx = positions[i].get_x() - x;

    float dy = positions[i].get_y() - y;

    return sqrtf(dx*dx+dy*dy);
}

void Map::merge_point(float x, float y,std::string color)
{
    size_t index = 999;

    float dis = 9999999;

    for(size_t i = 0;i<positions.size();i++)
    {
        if(color == positions[i].get_color() && dis < get_distance(x,y,i))
        {
            index = i;

            dis = get_distance(x,y,i);
        }
    }
    if(dis > 0.2 || index == 999)
    {
        position _position(x,y,color);

        positions.push_back(_position);
    }
    else
    {
        positions[index].update_information(x,y);

        positions[index].delay_empty();
    }
}

float Map::from_q_to_alpha(float w,float z)
{
    float alpha =  2*acosf(w)*180.0f/3.14f;

    if(z < 0)alpha*=-1;

    while(alpha < 0 )alpha+=360;

    while(alpha > 360)alpha-=360;
}
void Map::compute_line(float alpha, float beta)
{
    a1 = alpha + beta;

    a2 = alpha - beta;

    while(a1 < 0 )a1+=360;

    while(a1 > 360)a1-=360;

    while(a2 < 0 )a2+=360;

    while(a2 > 360)a2-=360;
}

void Map::update()
{
    for(size_t i = 0;i <positions.size();i++)
    {
        if(positions[i].visual_angle(robot_x,robot_y,a1,a2))
        {
            positions[i].delay_count();

            if(positions[i].delay_enough())
            {
                positions.erase(positions.begin()+i);

                i--;
            }
        }
        else
        {
            positions[i].delay_empty();
            continue;
        }
    }
}

size_t Map::position_to_pub()
{
    size_t  index = 99999;

    float distance = 99999.f;
    for(size_t i = 0;i < positions.size() ;i++)
    {

        if(positions[i].count_enough() == 0)continue;

        float dis = get_distance(robot_x,robot_y,i);

        if(dis < distance)
        {
            dis = distance;

            index = i;
        }
    }

    return index;
}

void Map::fellow()
{
    PointColors pointColors;

    PointColors temp;

    temp = Green.color_main(src);

    pointColors.insert(pointColors.end(),temp.begin(),temp.end());

    temp = Red.color_main(src);

    pointColors.insert(pointColors.end(),temp.begin(),temp.end());

    temp = Blue.color_main(src);

    pointColors.insert(pointColors.end(),temp.begin(),temp.end());

    for(size_t i = 0;i < pointColors.size();i++)
    {
        float x = pointColors[i].Point.x;

        float y = pointColors[i].Point.y;

        float z = get_depth(x,y);

        if(z < 0) continue;

        geometry_msgs::PointStamped origial_point,result_point;

        origial_point.header.frame_id = "camera_link";

        origial_point.header.stamp = ros::Time(0);

        //change the main axies
        origial_point.point.x =  z/100;

        origial_point.point.y = -x/100;

        origial_point.point.z = -y/1000;

        try
        {
            tf_Listener.transformPoint("map",origial_point,result_point);
        }
        catch (tf::TransformException &tf_ex)
        {
            ROS_ERROR("%s",tf_ex.what());

            continue;
        }

        if(judge_point((float)result_point.point.x,(float)result_point.point.y))
        {
            merge_point(x,y,pointColors[i].color_name);
        }
        else
        {
            continue;
        }
    }

    update();

    vwbot_controller::PoseAndColor msg;

    msg.pose.header.stamp.now();

    size_t _index = position_to_pub();

    if(_index > 100)
    {
        msg.pose.pose.position.x = -1;

        msg.pose.pose.position.y = -1;
    }
    else if(not_change == 0)
    {
        msg.pose.pose.position.x  = positions[_index].get_x();

        msg.pose.pose.position.y  = positions[_index].get_y();

        msg.color = (positions[_index].get_color()).c_str();
    }
    else
    {
        msg.pose.pose.position.x  = target_x;

        msg.pose.pose.position.y  = target_y;

        msg.color = target_color;
    }
    pub_location.publish(msg);


    if(target_color == "Green")
    {
        angle = Green.Alpha();
    }
    else if(target_color == "Red")
    {
        angle = Red.Alpha();
    }
    else if(target_color == "Blue")
    {
        angle = Blue.Alpha();
    }
    else
    {

    }

    std_msgs::Float32 Angle;

    Angle.data = angle;

    pub_angle.publish(Angle);
}