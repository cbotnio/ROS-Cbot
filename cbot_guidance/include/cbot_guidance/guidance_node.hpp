#ifndef GUIDANCE_NODE_H
#define GUIDANCE_NODE_H

#include <math.h>
#include <stdio.h>
#include <time.h> 
#include <std_msgs/Bool.h>

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_ros_msgs/WaypointInputs.h"
#include "cbot_ros_msgs/LineInputs.h"
#include "cbot_ros_msgs/ArcInputs.h"
#include "cbot_guidance/guidance.hpp"
#include "cbot_guidance/utm.hpp"

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <cbot_guidance/GuidanceConfig.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>


#define PI 3.1415926535
#define RAD_2_DEG 180.0/PI

namespace cbot_guidance{

class GuidanceNode {
    public:
        GuidanceNode(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh);

        ~GuidanceNode();

    private:
        Guidance guidance_;

        ros::NodeHandle nh_, private_nh_;


        ros::ServiceServer waypoint_server;
        ros::ServiceServer linefollow_server;
        ros::ServiceServer arcfollow_server;

        ros::Publisher controller_inputs_pub;
        ros::Publisher guidance_status_pub;
        
        ros::Subscriber ahrs_sub;
        
        ros::Timer timer;

        dynamic_reconfigure::ReconfigureRequest srv_req;
        dynamic_reconfigure::ReconfigureResponse srv_resp;
        dynamic_reconfigure::IntParameter int_param;
        dynamic_reconfigure::Config conf;

        dynamic_reconfigure::Server<cbot_guidance::GuidanceConfig> dyn_config_server_;
        
        void DynConfigCallback(cbot_guidance::GuidanceConfig &config, uint32_t level);

        void initializeParameters();
        bool initialized_parameters_;

        void navigationCallback(const geometry_msgs::Pose::ConstPtr& msg);
        
        bool waypointInputsCallback(cbot_ros_msgs::WaypointInputs::Request &req, cbot_ros_msgs::WaypointInputs::Response &res);
        bool linefollowInputsCallback(cbot_ros_msgs::LineInputs::Request &req, cbot_ros_msgs::LineInputs::Response &res);
        bool arcfollowInputsCallback(cbot_ros_msgs::ArcInputs::Request &req, cbot_ros_msgs::ArcInputs::Response &res);
        
        void timerCallback(const ros::TimerEvent& event);

        int guidance_on;

        double vehicle_pos_x, vehicle_pos_y;
        double desired_pos_x1,desired_pos_y1,desired_pos_x2,desired_pos_y2,desired_pos_xc,desired_pos_yc;
        double desired_heading, desired_thrust, nominal_velocity, desired_pitch;
        
        //Flag to be used for sending the 0 control inputs only once when guidance is turned off. 
        //This removes control from guidance and allows user to directly change controller inputs
        int flag;
        
        int parameter_flag;

        bool guidance_status;
        int guidance_mode;
        char Zone[20];
};

}

#endif