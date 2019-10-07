//
// Created by wang_shuai on 19-9-26.
//

#ifndef ROBSOTRACEVISUAL_KALMANGRID_H
#define ROBSOTRACEVISUAL_KALMANGRID_H

#include "GeneralSupport.h"

class position
{
private:

    std::string color;

    float x;

    float y;


    int delay = 0;//to record how long the visual angle has contained the position

    int count = 0;//the num of detected point belongs to this area;

    float dx1 = 0;

    float dx2 = 0;

    float dy1 = 0;

    float dy2 = 0;
public:
    position(float x,float y,std::string color);

    void update_information(float x,float y);

    void delay_empty();

    void delay_count();

    bool delay_enough();

    bool count_enough();

    bool visual_angle(float x0,float y0,float angle1,float angle2);//robot position ,up visual angle, low visual angle

    float get_angle(float x0,float y0);

    float get_x();

    float get_y();

    std::string get_color();
};

typedef std::vector<position> Positions;

#endif //ROBSOTRACEVISUAL_KALMANGRID_H
