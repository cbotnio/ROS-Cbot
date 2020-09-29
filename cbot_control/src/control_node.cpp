#include "ros/ros.h"
#include "cbot_ros_msgs/AHRS.h"
#include "nav_msgs/Odometry.h"
#include "cbot_ros_msgs/ControllerSettings.h"
#include "cbot_ros_msgs/ThrusterControl.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_control/controllers.hpp"

#define PI 3.141592653
#define deg2rad 0.0174532925
#define rad2deg 57.2957795

double desired_heading, desired_thrust, desired_pitch, desired_depth=1, desired_u=0;
double yaw, yaw_rate, pitch, pitch_rate,roll=0, last_pitch, last_pitch_rate, Ts, depth,Vx,Vy,Vz;
double common_mode_F = 0, differential_mode_F = 0;
double common_mode_V = 0, differential_mode_V = 0;
int Controller_ON, HeadingCtrlON, PitchCtrlON, VelocityCtrlON, DepthCtrlON, first_diff_pitch=0, flag=0;


void ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg)
{
    if(msg->AHRS_Status == 2)
    {
        yaw = msg->YawAngle;
        yaw_rate = msg->YawRate;
        pitch = msg->Pitch;
        pitch_rate = msg->PitchRate;
    }
}

void pose_callback(const nav_msgs::Odometry::ConstPtr& msg){
    depth = -msg->pose.pose.position.z;
    Vx = msg->twist.twist.linear.y;
    Vy = msg->twist.twist.linear.x;
    Vz = -msg->twist.twist.linear.z;
}

double getBodyVel(){
    double p = pitch*deg2rad, y = yaw*deg2rad;
    return (cos(p)*cos(y)*Vx + sin(y)*cos(p)*Vy - sin(p)*Vz);
}

bool controllerSettingsCallback(cbot_ros_msgs::ControllerSettings::Request &req, cbot_ros_msgs::ControllerSettings::Response &res)
{
    if(req.update_settings)
    {
        ros::param::set("yaw_lqr_kyaw", req.yaw_lqr_kyaw);
        ros::param::set("yaw_lqr_kr", req.yaw_lqr_kr);
        res.yaw_lqr_kyaw = req.yaw_lqr_kyaw;
        res.yaw_lqr_kr = req.yaw_lqr_kr;
    }

    return true;
}

bool controllerInputsCallback(cbot_ros_msgs::ControllerInputs::Request &req, cbot_ros_msgs::ControllerInputs::Response &res)
{
    desired_heading = req.desired_heading;
    desired_thrust = req.desired_thrust;
    desired_pitch = req.desired_pitch;  
    desired_depth = req.desired_depth;
    desired_u = req.desired_u;
    res.desired_heading = desired_heading;
    res.desired_thrust = desired_thrust;
    res.desired_pitch = desired_pitch;
    res.desired_depth = desired_depth;
    res.desired_u = desired_u;
    return true;
}


void timerCallback(const ros::TimerEvent& event)
{
    ros::param::get("Controller_ON", Controller_ON);
    ros::param::get("HeadingCtrl", HeadingCtrlON);
    ros::param::get("PitchCtrl", PitchCtrlON);
    ros::param::get("VelocityCtrl", VelocityCtrlON);
    ros::param::get("DepthCtrl", DepthCtrlON);
    double u = getBodyVel();
    if(Controller_ON)
    {
        flag=0;
        // common_mode_F = desired_thrust;
        if(HeadingCtrlON)
            differential_mode_F = CONTROLLERS::lqr_yaw(desired_heading, yaw, yaw_rate, Ts);
        else
            differential_mode_F = 0;
        if(PitchCtrlON)
            differential_mode_V = CONTROLLERS::lqr_pitch(desired_pitch, pitch, pitch_rate, Ts);
        else
            differential_mode_V = 0;
        if(DepthCtrlON)
            common_mode_V = CONTROLLERS::depth(desired_depth, depth, Ts);
        else
            common_mode_V = 0;
        if(VelocityCtrlON)
            common_mode_F = CONTROLLERS::velocity(desired_u, u, Ts);
        else
            common_mode_F = 0;

        // printf("u: %f | CF %f | DF %f | CV %f | DV %f\n",u, common_mode_F, differential_mode_F,common_mode_V, differential_mode_V);
    
        cbot_ros_msgs::ThrusterControl temp;
        temp.request.comm_mode_F = common_mode_F;
        temp.request.diff_mode_F = differential_mode_F;
        temp.request.comm_mode_V = common_mode_V;
        temp.request.diff_mode_V = differential_mode_V;
        temp.request.update = 1;
        
        if (ros::service::call("thruster_control", temp))
        {
        }
    }
    else if(!Controller_ON && flag==0){
        flag=1;
        cbot_ros_msgs::ThrusterControl temp;
        temp.request.comm_mode_F = 0;
        temp.request.diff_mode_F = 0;
        temp.request.comm_mode_V = 0;
        temp.request.diff_mode_V = 0;
        temp.request.update = 1;
    }
}

ros::ServiceServer controller_settings_server, controller_inputs_server;
ros::ServiceClient thruster_control_client;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_node");
    ros::Time::init();
    ros::NodeHandle n;
    n.getParam("Controller_ON", Controller_ON);
    n.getParam("Ts_controller", Ts);

    controller_settings_server = n.advertiseService("controller_settings", controllerSettingsCallback);
    controller_inputs_server = n.advertiseService("controller_inputs", controllerInputsCallback);
    thruster_control_client = n.serviceClient<cbot_ros_msgs::ThrusterControl>("thruster_control");

    ros::Subscriber ahrs_sub = n.subscribe("AHRS", 1, ahrsCallback);
    ros::Subscriber pose_sub = n.subscribe("/cbot/pose_gt", 1, pose_callback);

    ros::Timer timer = n.createTimer(ros::Duration(Ts), timerCallback);

    ros::spin();
    return  0;
}