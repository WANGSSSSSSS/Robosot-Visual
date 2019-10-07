//
// Created by wang_shuai on 19-9-26.
//
#include "../include/RobsotRaceVisual/KalmanGrid.h"


position::position(float x, float y,std::string color)
{
    this->x = x;

    this->y = y;

    this->color = color;
}
void position::delay_count()
{
    delay++;
}
void position::delay_empty()
{
    delay = 0;
}
bool position::delay_enough()
{
    return delay >= 6;
}
bool position::count_enough()
{
    return count > 5;
}

// 目标位置更新
// kalman so hard that i use pid
void position::update_information(float x, float y)
{
    this->x = 0.5f*this->x + 0.5f*x +dx1+dx2;

    this->y = 0.5f*this->y + 0.5f*y +dy1+dy2;

    dy2 = dy1;

    dx2 = dx1;

    dx1 = (x - this->x)*0.2;
    
    dy1 = (y - this->y)*0.2;

    count ++;
}

float position::get_angle(float x0, float y0)
{
    //hahahahha

    //wo ai chi shi shi

    //how about eating shit for launch
}

float position::get_x()
{
    return x;
}

float position::get_y()
{
    return y;
}

std::string position::get_color()
{
    return color;
}
//计算视角
bool position::visual_angle(float x0, float y0, float angle1, float angle2)
{
    float diff_x = x - x0;

    float diff_y = y - y0;

    float angle = atan2f(diff_y,diff_x)/3.14f*180.f;

    if(angle < 0) angle +=360;

    if(angle > 360) angle -=360;

    if(angle1 > angle2)
    {
        return angle > angle1 || angle < angle2;
    }
    else
    {
        return angle > angle1 && angle < angle2;
    }
}
