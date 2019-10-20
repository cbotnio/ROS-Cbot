#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "cbot_ros_msgs/GuidanceInputs.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_guidance/guidance.hpp"

double desired_heading, desired_thrust, nominal_velocity;
bool guidance_status;

double vehicle_pos_x, vehicle_pos_y;
int guidance_on;
int guidance_mode;

void navigationCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    vehicle_pos_x = msg->position.x;
    vehicle_pos_y = msg->position.y;
}

bool guidanceInputsCallback(cbot_ros_msgs::GuidanceInputs::Request &req, cbot_ros_msgs::GuidanceInputs::Response &res)
{
    if(req.update_inputs)
    {
        guidance_on = req.guidance_on;
        guidance_mode = req.guidance_mode;
        GUIDANCE::desired_pos_x1 = req.desired_pos_x1;
        GUIDANCE::desired_pos_y1 = req.desired_pos_y1;
        GUIDANCE::desired_pos_x2 = req.desired_pos_x2;
        GUIDANCE::desired_pos_y2 = req.desired_pos_y2;
        GUIDANCE::desired_pos_xc = req.desired_pos_xc;
        GUIDANCE::desired_pos_yc = req.desired_pos_yc;
        GUIDANCE::arc_follow_direction = req.arc_follow_direction;
        nominal_velocity = req.nominal_velocity;
    }
    else
    {
        req.guidance_on = guidance_on;
        req.guidance_mode = guidance_mode; 
        req.desired_pos_x1 = GUIDANCE::desired_pos_x1;
        req.desired_pos_y1 = GUIDANCE::desired_pos_y1;
        req.desired_pos_x2 = GUIDANCE::desired_pos_x2;
        req.desired_pos_y2 = GUIDANCE::desired_pos_y2;
        req.desired_pos_xc = GUIDANCE::desired_pos_xc;
        req.desired_pos_yc = GUIDANCE::desired_pos_yc;
        req.arc_follow_direction = GUIDANCE::arc_follow_direction;
        req.nominal_velocity = nominal_velocity;

        if(guidance_status)
            res.guidance_status = 1;
        else
            res.guidance_status = 0;
    }

    return true;
}

void timerCallback(const ros::TimerEvent& event)
{
    if(guidance_on)
    {
        desired_thrust = nominal_velocity * 4.5;
        if(guidance_mode == 0)
            guidance_status = GUIDANCE::WayPtGuidance(vehicle_pos_x, vehicle_pos_y);
        else if(guidance_mode == 1)
            guidance_status = GUIDANCE::LineFollowGuidance(vehicle_pos_x, vehicle_pos_y);
        else if(guidance_mode == 2)
            guidance_status = GUIDANCE::LineFollowGuidance(vehicle_pos_x, vehicle_pos_y);

        if(guidance_status)
        {
            guidance_on = 0;
            desired_thrust = 0;
        }

        cbot_ros_msgs::ControllerInputs temp;
        temp.request.desired_heading = desired_heading;
        temp.request.desired_thrust = desired_thrust;
        temp.request.update_inputs = 1;

        if (ros::service::call("controller_inputs", temp))
        {
        }
    }
}

ros::ServiceServer guidance_inputs_server;
ros::ServiceClient controller_inputs_client;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "guidance_node");
    ros::Time::init();
    ros::NodeHandle n;

    guidance_inputs_server = n.advertiseService("guidance_inputs", guidanceInputsCallback);
    controller_inputs_client = n.serviceClient<cbot_ros_msgs::ControllerInputs>("controller_inputs");

    ros::Subscriber ahrs_sub = n.subscribe("NAVIGATION", 1000, navigationCallback);  

    ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

    ros::spin();

    return  0;

}
