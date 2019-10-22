#include "cbot_control/controllers.hpp"

double CONTROLLERS::yaw_lqr_kyaw = 1.0;
double CONTROLLERS::yaw_lqr_kr = 1.8;

float CONTROLLERS::lqr_yaw(double desired_heading, double yaw, double yaw_rate)
{
    float err = 0, delta;

    //choose the smaller angle for rotation
    if (yaw < 0) yaw = 360 - fabs(yaw);
    err = ( desired_heading - yaw );
    if (err > 180) 
        err = err - 360;
    else if (err < -180) 
        err = 360 + err;

    delta = (err * yaw_lqr_kyaw) - (yaw_rate * yaw_lqr_kr); 

    if (delta > 10)  delta = 10;
    if (delta < -10) delta = -10;

    return delta;
}