//
// Created by wang_shuai on 19-9-28.
//

#include "../include/RobsotRaceVisual/ResultMap.h"


int main(int argc ,char** argv)
{
    ros::init(argc,argv,NODE_NAMESPACE);

    Map resultMap;

    ros::Rate clock(1000);

    while(resultMap.nh.ok())
    {
        resultMap.fellow();

        ros::spinOnce();//很重要，开启回调函数

        clock.sleep();
    }
}

