#include <stdio.h>
#include "geometry_msgs/Pose.h"
#include "cbot_ros_msgs/ControllerInputs.h"
#include "cbot_ros_msgs/GuidanceInputs.h"
#include "cbot_guidance/guidance.hpp"
#include <time.h> 
#include <std_msgs/Bool.h>
#include "cbot_guidance/utm.hpp"

int guidanceON = 0;

double vehicle_pos_x, vehicle_pos_y;
double desired_heading, desired_thrust, nominal_velocity, desired_pitch;
double desired_pos_x1,desired_pos_y1,desired_pos_x2,desired_pos_y2;
bool guidance_status;
int guidance_mode=-1;
char Zone[20];

ros::ServiceServer guidance_inputs_server;
ros::ServiceClient controller_inputs_client;
ros::Publisher guidanceStatus_pub;


void navigationCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    vehicle_pos_x = msg->position.x;
    vehicle_pos_y = msg->position.y;
}

bool guidanceInputsCallback(cbot_ros_msgs::GuidanceInputs::Request &req, cbot_ros_msgs::GuidanceInputs::Response &res)
{
    printf("New Goal\n");
    
    guidance_mode = req.guidance_mode;
    UTM::LLtoUTM(23, req.desired_pos_x1, req.desired_pos_y1, &desired_pos_x1, &desired_pos_y1, Zone);
    UTM::LLtoUTM(23, req.desired_pos_x2, req.desired_pos_y2, &desired_pos_x2, &desired_pos_y2, Zone);
    
    GUIDANCE::desired_pos_x1 = desired_pos_x1;
    GUIDANCE::desired_pos_y1 = desired_pos_y1;
    GUIDANCE::desired_pos_x2 = desired_pos_x2;
    GUIDANCE::desired_pos_y2 = desired_pos_y2;
    GUIDANCE::desired_pos_xc = req.desired_pos_xc;
    GUIDANCE::desired_pos_yc = req.desired_pos_yc;
    GUIDANCE::arc_follow_direction = req.arc_follow_direction;
    nominal_velocity = req.nominal_velocity;

    res.update_inputs = true;
}

void timerCallback(const ros::TimerEvent& event)
{
    ros::param::get("/GUIDANCE_ON", guidanceON);
    ros::param::set("HeadingCtrl", 0);
    ros::param::set("PitchCtrl", 0);
    ros::param::set("VelocityCtrl", 0);
    ros::param::set("DepthCtrl", 0);
    if(guidanceON)
    {
        desired_thrust = nominal_velocity * 4.5;
        if(guidance_mode == 0){
            ros::param::set("HeadingCtrl", 1);
            ros::param::set("VelocityCtrl", 1);
            guidance_status = GUIDANCE::WayPtGuidance(vehicle_pos_x, vehicle_pos_y);
        }
        else if(guidance_mode == 1){
            ros::param::set("HeadingCtrl", 1);
            ros::param::set("VelocityCtrl", 1);
            guidance_status = GUIDANCE::LineFollowGuidance(vehicle_pos_x, vehicle_pos_y, 1);
        }
        else if(guidance_mode == 2)
            guidance_status = GUIDANCE::ArcFollowGuidance(vehicle_pos_x, vehicle_pos_y);
        else if(guidance_mode == 3){
            ros::param::set("HeadingCtrl", 1);
            ros::param::set("VelocityCtrl", 1);
            guidance_status = GUIDANCE::StKpGuidance(vehicle_pos_x, vehicle_pos_y);
            nominal_velocity = GUIDANCE::desiredThrustCMF;
        }
        else if(guidance_mode == -1){
            desired_thrust = 0;
            desired_heading = 0;
            desired_pitch = 0;
            nominal_velocity = 0;
        }

        if(guidance_status){
            desired_thrust = 0;
            nominal_velocity = 0;
            guidanceON = 0;
        }
        
        std_msgs::Bool reached;
        reached.data = (guidance_status)?true:false;
        guidanceStatus_pub.publish(reached);

        cbot_ros_msgs::ControllerInputs temp;
        temp.request.desired_heading = GUIDANCE::desired_heading;
        temp.request.desired_thrust = desired_thrust;
        temp.request.desired_u = nominal_velocity;
        temp.request.desired_pitch = 0;
        temp.request.desired_depth = 1;
        
        if (ros::service::call("controller_inputs", temp))
        {
            // printf("controller service called successfully\n");
        }
    }
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "guidance_node");
    ros::Time::init();

    ros::NodeHandle n;
    n.getParam("/GUIDANCE_ON", guidanceON);

    guidance_inputs_server = n.advertiseService("guidance_inputs", guidanceInputsCallback);
    controller_inputs_client = n.serviceClient<cbot_ros_msgs::ControllerInputs>("controller_inputs");
    
    ros::Subscriber ahrs_sub = n.subscribe("/position", 5, navigationCallback);
    guidanceStatus_pub = n.advertise<std_msgs::Bool>("/guidanceStatus", 5);
    ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

    ros::spin();
    return  0;
}
