//
// Created by wang_shuai on 19-9-26.
//

#ifndef ROBSOTRACEVISUAL_RESULTMAP_H
#define ROBSOTRACEVISUAL_RESULTMAP_H

#include "GeneralSupport.h"
#include "KalmanGrid.h"
#include "ColorSupport.h"

class Map
{
private:

    tf::TransformListener tf_Listener;

    ros::Subscriber sub_src;

    ros::Subscriber sub_range_mat;

    ros::Subscriber sub_change;

    ros::Subscriber sub_occupied_grid;

    ros::Subscriber sub_robot_infomation;

    ros::Publisher pub_angle;

    ros::Publisher pub_location;

    //RobsotRaceVisual::dy_color dy_server;

    nav_msgs::OccupancyGrid OccupancyGrid;

    cv::Mat range_mat;

    cv::Mat src;

    RosColor Green;

    RosColor Red;

    RosColor Blue;

    Positions positions;

    float angle = -9999.9f;

    float a1 = 0;

    float a2 = 0;

    float robot_x;

    float robot_y;

    float target_x;

    float target_y;

    std::string target_color;

    bool not_change = 0;

    float bounder_left = 0;

    float bounder_right = 500;

    float bounder_up = 500;

    float bounder_down = 0;

public:

    ros::NodeHandle nh;

    Map();

    void update_change(std_msgs::Bool::ConstPtr msg);

    void update_occupied_grid(nav_msgs::OccupancyGrid::ConstPtr msg);

    void update_range_mat(sensor_msgs::ImageConstPtr msg);

    void update_src(sensor_msgs::ImageConstPtr msg);

    void update_robot_information(geometry_msgs::PoseStamped msg);

    float get_depth(float x,float y);

    bool judge_point(float x, float y);

    void merge_point(float x, float y,std::string color);

    void update();

    size_t position_to_pub();

    float get_distance(float x, float y,size_t i);

    float from_q_to_alpha(float w,float z);

    void compute_line(float alpha, float beta);

    void fellow();

};
#endif //ROBSOTRACEVISUAL_RESULTMAP_H
