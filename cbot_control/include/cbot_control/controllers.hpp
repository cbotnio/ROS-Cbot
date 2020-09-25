#ifndef _CONTROLLERS_H_
#define _CONTROLLERS_H_

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI 3.141592653
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

class CONTROLLERS 
{

    public:
    	//Heading Control
    	static double prev_yaw, prev_yawRate, yawAW, yawOut;
        static float lqr_yaw(double desired_heading, double yaw, double yawRate, double Ts);
    	
    	//Pitch Control
    	static double prev_pitch, prev_pitchRate, pitchAW, pitchOut;
        static double lqr_pitch(double desired_pitch, double pitch, double pitchRate, double Ts);

        //Depth Control 
    	static double err3Y, UY, int_depth_err, int_u_err; 
    	static double depth(double desired_depth, double depth, double Ts);
    	static float velocity(double desired_u, double u, double Ts);
    private:
  
};

#endif