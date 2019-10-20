#ifndef _CONTROLLERS_H_
#define _CONTROLLERS_H_

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

class CONTROLLERS 
{

    public:
        static double yaw_lqr_kyaw, yaw_lqr_kr;
        static float lqr_yaw(double desired_heading, double yaw, double yaw_rate);
    
    private:
  
};

#endif