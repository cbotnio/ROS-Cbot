#include "ros/ros.h"
#include "asv_ros_revamp/AHRS.h"
#include "asv_ros_revamp/controller_settings.h"
#include "asv_ros_revamp/thruster_control.h"
#include "asv_ros_revamp/controller_inputs.h"
#include "asv_ros_revamp/control/controllers.hpp"

double desired_heading, desired_thrust;
double yaw, yaw_rate;
double common_mode = 0, differential_mode = 0;
int Controller_ON;


void ahrs_callback(const asv_ros_revamp::AHRS::ConstPtr& msg)
{
  if(msg->AHRS_Status == 2)
  {
    yaw = msg->YawAngle;
    yaw_rate = msg->YawRate;
  }
}

bool controller_settings_callback(asv_ros_revamp::controller_settings::Request &req, asv_ros_revamp::controller_settings::Response &res)
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

bool controller_inputs_callback(asv_ros_revamp::controller_inputs::Request &req, asv_ros_revamp::controller_inputs::Response &res)
{
  if(req.update_inputs)
  {
    desired_heading = req.desired_heading;
    desired_thrust = req.desired_thrust;  //is it good idea to update desired thrust everytime?
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

void timer_callback(const ros::TimerEvent& event)
{
  if(Controller_ON)
  {
    common_mode = desired_thrust;
    // ROS_INFO("Desired heading %f | yaw %f", desired_heading, yaw);
    differential_mode = CONTROLLERS::lqr_yaw(desired_heading, yaw, yaw_rate);

    asv_ros_revamp::thruster_control temp;
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
  ros::init(argc, argv, "CONTROL_NODE");
  ros::Time::init();
  ros::NodeHandle n;

  ros::param::getCached("Controller_ON", Controller_ON); //This will not work, if u want to switch in betwn

  controller_settings_server = n.advertiseService("controller_settings", controller_settings_callback);
  controller_inputs_server = n.advertiseService("controller_inputs", controller_inputs_callback);
  thruster_control_client = n.serviceClient<asv_ros_revamp::thruster_control>("thruster_control");

  ros::Subscriber ahrs_sub = n.subscribe("AHRS", 1000, ahrs_callback);  

  ros::Timer timer = n.createTimer(ros::Duration(0.1), timer_callback);

  ros::spin();

  return  0;

}
