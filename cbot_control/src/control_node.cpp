#include "ros/ros.h"
#include "cbot_ros_msgs/AHRS.h"
#include "cbot_ros_msgs/ControllerSettings.h"
#include "cbot_ros_msgs/ThrusterControl.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_control/controllers.hpp"

double desired_heading, desired_thrust;
double yaw, yaw_rate;
double common_mode = 0, differential_mode = 0;
int Controller_ON;


void ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg)
{
    if(msg->AHRS_Status == 2)
    {
        yaw = msg->YawAngle;
        yaw_rate = msg->YawRate;
    }
}

bool controllerSettingsCallback(cbot_ros_msgs::ControllerSettings::Request &req, cbot_ros_msgs::ControllerSettings::Response &res)
{
    if(req.update_settings)
    {
        CONTROLLERS::yaw_lqr_kyaw = req.yaw_lqr_kyaw;
        CONTROLLERS::yaw_lqr_kr = req.yaw_lqr_kr;
        res.yaw_lqr_kyaw = CONTROLLERS::yaw_lqr_kyaw;
        res.yaw_lqr_kr = CONTROLLERS::yaw_lqr_kr;
    }
    else
    {
        res.yaw_lqr_kyaw = CONTROLLERS::yaw_lqr_kyaw;
        res.yaw_lqr_kr = CONTROLLERS::yaw_lqr_kr;
    }

    return true;
}

bool controllerInputsCallback(cbot_ros_msgs::ControllerInputs::Request &req, cbot_ros_msgs::ControllerInputs::Response &res)
{
    if(req.update_inputs)
    {
        desired_heading = req.desired_heading;
        desired_thrust = req.desired_thrust;  
        res.desired_heading = desired_heading;
        res.desired_thrust = desired_thrust;
    }
    else
    {
      res.desired_heading = CONTROLLERS::yaw_lqr_kyaw;
      res.desired_thrust = CONTROLLERS::yaw_lqr_kr;
    }

    return true;
}

void timerCallback(const ros::TimerEvent& event)
{
    if(Controller_ON)
    {
        common_mode = desired_thrust;
        // ROS_INFO("Desired heading %f | yaw %f", desired_heading, yaw);
        differential_mode = CONTROLLERS::lqr_yaw(desired_heading, yaw, yaw_rate);

        cbot_ros_msgs::ThrusterControl temp;
        temp.request.comm_mode = common_mode;
        temp.request.diff_mode = differential_mode;
        temp.request.update = 1;

        if (ros::service::call("thruster_control", temp))
        {
        }

        // ROS_INFO("Commond mode %f | Differential mode %f", common_mode, differential_mode);
    }
}

ros::ServiceServer controller_settings_server, controller_inputs_server;
ros::ServiceClient thruster_control_client;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::NodeHandle n;

    ros::param::getCached("Controller_ON", Controller_ON); //This will not work, if u want to switch in betwn

    controller_settings_server = n.advertiseService("controller_settings", controllerSettingsCallback);
    controller_inputs_server = n.advertiseService("controller_inputs", controllerInputsCallback);
    thruster_control_client = n.serviceClient<cbot_ros_msgs::ThrusterControl>("thruster_control");

    ros::Subscriber ahrs_sub = n.subscribe("AHRS", 1000, ahrsCallback);  

    ros::Timer timer = n.createTimer(ros::Duration(0.1), timerCallback);

    ros::spin();
    return  0;
}
