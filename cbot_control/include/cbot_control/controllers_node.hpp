#ifndef CONTROLLERS_NODE_H
#define CONTROLLERS_NODE_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <Eigen/Eigen>
#include <boost/bind.hpp>

//ROS
#include "ros/ros.h"
#include "cbot_ros_msgs/AHRS.h"
#include "nav_msgs/Odometry.h"
#include "cbot_ros_msgs/ControllerSettings.h"
#include "cbot_ros_msgs/ThrusterCMDM.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_control/controllers.hpp"

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <cbot_control/ControllersConfig.h>

#define PI 3.141592653
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

namespace cbot_control {

class ControllersNode{

    public:
    
        ControllersNode(const ros::NodeHandle& nh, const ros::NodeHandle& private_nh);
    
        ~ControllersNode();

    private:
        Controllers controllers_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        dynamic_reconfigure::Server<cbot_control::ControllersConfig> dyn_config_server_;
        void DynConfigCallback(cbot_control::ControllersConfig &config, uint32_t level);

        ros::Publisher thruster_cmdm_pub;
        ros::Subscriber controller_inputs_sub;
        ros::Subscriber ahrs_sub;
        ros::Subscriber pose_sub;
        ros::Timer timer;
        
        void ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg);
        
        void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
        
        double getBodyVel();
        
        void controllerInputsCallback(const cbot_ros_msgs::ControllerInputs::ConstPtr& msg);
        
        void timerCallback(const ros::TimerEvent& event);


        double desired_heading, desired_thrust, desired_pitch, desired_depth, desired_u;
        double yaw, yaw_rate, pitch, pitch_rate,roll, last_pitch, last_pitch_rate, Ts, depth, vx, vy, vz;
        double common_mode_F, differential_mode_F;
        double common_mode_V, differential_mode_V;
        int controller_on, heading_ctrl_on, pitch_ctrl_on, speed_ctrl_on, depth_ctrl_on, first_diff_pitch, flag;
};

} // end namespace cbot_control

#endif