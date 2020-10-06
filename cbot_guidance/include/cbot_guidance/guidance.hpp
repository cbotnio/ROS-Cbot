#ifndef GUIDANCE_H
#define GUIDANCE_H

#include <math.h>
#include "stdio.h"
#include "ros/ros.h"
#include "cbot_guidance/matrix.hpp"

#define PI 3.1415926535
#define RAD_2_DEG 180.0/PI

namespace cbot_guidance{

class Guidance {
    public:

        Guidance(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);

        ~Guidance();
    
        bool WayPtGuidance(double veh_x, double veh_y);
        bool LineFollowGuidance(double veh_x, double veh_y, double Ts);
        bool StKpGuidance(double veh_x, double veh_y);
        bool ArcFollowGuidance(double veh_x, double veh_y);

        void setDesiredX1(double x1){
            desired_pos_x1 = x1;
        }
        void setDesiredY1(double y1){
            desired_pos_y1 = y1;
        }
        void setDesiredX2(double x2){
            desired_pos_x2 = x2;
        }
        void setDesiredY2(double y2){
            desired_pos_y2 = y2;
        }
        void setDesiredXc(double xc){
            desired_pos_xc = xc;
        }
        void setDesiredYc(double yc){
            desired_pos_yc = yc;
        }
        void setArcDirection(int afd){
            arc_follow_direction = afd;
        }
        double getDesiredHeading(){
            return desired_heading;
        }
        double getDesiredSpeed(){
            return desired_thrust_cmf;
        }

        //Dymanic Parameters
        void setLFWKp(double lfw_kp_){
            lfw_kp = lfw_kp_;
        }
        void setLFWKd(double lfw_kd_){
            lfw_kd = lfw_kd_;
        }
        void setARCKp(double arc_kp_){
            arc_kp = arc_kp_;
        }
        void setARCKd(double arc_kd_){
            arc_kd = arc_kd_;
        }

    private:
        ros::NodeHandle nh_, private_nh_;

        void initializeParameters();
        bool initialized_parameters_;

        double desired_pos_x1, desired_pos_y1, desired_pos_x2, desired_pos_y2, desired_pos_xc, desired_pos_yc;
        
        // Waypoint Guidance
        double desired_heading, acceptable_radius;
        
        //Line Follow Guidance
        double last_d, lfw_psi_c, lfw_antiwindup, lfw_kd, lfw_kp;

        //Station Keeping
        double desired_thrust_cmf;

        //Arc Following
        int arc_follow_direction;
        double arc_psi_c, arc_kd, arc_kp, antiwindup_arc;
};

}

#endif