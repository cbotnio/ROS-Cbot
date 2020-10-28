#ifndef CONTROLLERS_H
#define CONTROLLERS_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Eigen/Eigen>

#define PI 3.141592653
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

namespace cbot_control {

class Controllers{

    public:
    
        Controllers(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    
        ~Controllers();

        //Dymanic Parameters
        void setYawGains(const Eigen::VectorXd& k_yaw){
            yaw_k = k_yaw(0);
            yaw_kr = k_yaw(1);
            yaw_ki = k_yaw(2);
            yaw_kaw = k_yaw(3);
        }
        void setYawDMSaturation(double yaw_dm_sat){
            yaw_dm_saturation = yaw_dm_sat;
        }
        void setPitchGains(const Eigen::VectorXd& k_pitch){
            pitch_k = k_pitch(0);
            pitch_kr = k_pitch(1);
            pitch_ki = k_pitch(2);
            pitch_kaw = k_pitch(3);
        }
        void setPitchDMSaturation(double pitch_dm_sat){
            pitch_dm_saturation = pitch_dm_sat;
        }
        void setDepthGains(const Eigen::Vector2d& k_depth){
            depth_k = k_depth(0);
            depth_ki = k_depth(1);
        }
        void setDepthCMSaturation(double depth_cm_sat){
            depth_cm_saturation = depth_cm_sat;
        }
        void setSpeedGains(const Eigen::Vector2d& k_speed){
            speed_k = k_speed(0);
            speed_ki = k_speed(1);
        }
        void setSpeedCMSaturation(double speed_cm_sat){
            speed_cm_saturation = speed_cm_sat;
        }

    	//Heading Control
        double lqrYaw(double desired_heading, double yaw, double yaw_rate, double Ts);
    	
    	//Pitch Control
        double lqrPitch(double desired_pitch, double pitch, double pitch_rate, double Ts);

        //Depth Control 
    	double depth(double desired_depth, double depth, double Ts);
    	
        //Velocity Control
        double velocity(double desired_u, double u, double Ts);

    private:
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        void initializeParameters();

        bool initialized_parameters_;

        //Heading Control parameters
    	double prev_yaw, prev_yaw_rate, yaw_aw, yaw_out;
        double yaw_k, yaw_kr, yaw_ki, yaw_kaw, yaw_dm_saturation;

        //Pitch Control parameters
    	double prev_pitch, prev_pitch_rate, pitch_aw, pitch_out;
        double pitch_k, pitch_kr, pitch_ki, pitch_kaw, pitch_dm_saturation;

        //Depth Control parameters
        double depth_k, depth_ki, int_depth_err, depth_cm_saturation;

        //Velocity Control Paratmeters
        double  speed_k, speed_ki, int_u_err, speed_cm_saturation;
};

} // end namespace cbot_control

#endif