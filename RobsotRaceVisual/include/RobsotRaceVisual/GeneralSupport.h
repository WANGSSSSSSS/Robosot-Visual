//
// Created by wang_shuai on 19-9-26.
//

#ifndef ROBSOTRACEVISUAL_GENERALSUPPORT_H
#define ROBSOTRACEVISUAL_GENERALSUPPORT_H


#define DEBUG_MODE
#define RUN_MODE

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <tf/transform_listener.h>
#include <cmath>
#include "PoseAndColor.h"
#include <dynamic_reconfigure/server.h>
#include <RobsotRaceVisual/dy_colorConfig.h>
#include <yaml-cpp/yaml.h>
#include <iostream>

struct PointColor
{
    cv::Point2f Point;

    std::string color_name;
};

typedef std::vector<cv::Point> contour;

typedef std::vector<contour>   contours;

typedef std::vector<cv::Rect>  Rects;

typedef std::vector<cv::Point2f> Points;

typedef std::vector<PointColor> PointColors;

typedef dynamic_reconfigure::Server<RobsotRaceVisual::dy_colorConfig> dy_colorServer;



#define  SRC_SUBSCRIBE_TOPIC "a"

#define  DEP_SUBSCRIBE_TOPIC "b"

#define  ROB_SUBSCRIBE_TOPIC "c"

#define  GRID_SUBSCRIBE_TOPIC "d"

#define  CHANGE_SUBSCRIBE_TOPIC "e"

#define  NODE_NAMESPACE "node_ns"

#define  ANGLE_PUBLISH_TOPIC "f"

#define  LOCAT_PUBLISH_TOPIC "g"

#define  WRITER_PATH "/home/wang_shuai/catkin_ws/src/RobsotRaceVisual/param/"

static char total = '0';


#endif //ROBSOTRACEVISUAL_GENERALSUPPORT_H
