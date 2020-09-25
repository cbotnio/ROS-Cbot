#include "ros/ros.h"
#include "cbot_ros_msgs/SensorsStatus.h"
#include "cbot_ros_msgs/GPS.h"
#include "cbot_ros_msgs/AHRS.h"

#define maxRoll 25 //degrees
#define maxPitch 25 //degrees


double roll, rollRate, pitch, pitchRate, yaw, yawRate;
int AHRS_status, GPS_status;

void gpsCallback(const cbot_ros_msgs::GPS::ConstPtr& msg)
{
    GPS_status = msg->GPS_status;
}

void ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg)
{
    roll = msg->Roll;
    pitch = msg->Pitch;
    yaw = msg->YawAngle;
    rollRate = msg->RollRate;
    pitchRate = msg->PitchRate;
    yawRate = msg->YawRate;
    AHRS_status = msg->AHRS_Status;
}

void sensorsStatusCallback(const cbot_ros_msgs::SensorsStatus::ConstPtr& msg)
{
    if(msg->AHRS_OK)
    {
        if(!AHRS_status)
            ROS_WARN("[SAFETY NODE] AHRS FAIL");
    }
    else
    {
        ROS_WARN("[SAFETY NODE] AHRS FAIL");
    }

    if(msg->GPS_OK)
    {
        if(GPS_status == 1)
            ROS_WARN("[SAFETY NODE] NO GPS FIX");
        else if (GPS_status == 0)
            ROS_WARN("[SAFETY NODE] GPS FAIL 2");
    }
    else
    {
        ROS_WARN("[SAFETY NODE] GPS FAIL");
    }
}

void checkRotations(){
    if(fabs(roll)>maxRoll){
        ROS_WARN("[SAFETY NODE] ROLL ANGLE REACHED LIMIT \n TURNING OFF THRUSTERS");
        ros::param::set("/HIL",0);
    }
    if(fabs(pitch)>maxPitch){
        ROS_WARN("[SAFETY NODE] PITCH ANGLE REACHED LIMIT \n TURNING OFF THRUSTERS");
        ros::param::set("/HIL",0);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "safety_monitor_node");
    ros::Time::init();
    ros::NodeHandle n;

    ros::Subscriber gps_sub = n.subscribe("/GPS", 1, gpsCallback);  
    ros::Subscriber ahrs_sub = n.subscribe("/AHRS", 1, ahrsCallback);  
    ros::Subscriber sensor_sub = n.subscribe("/SENSOR", 1, sensorsStatusCallback);  

    ros::spin();
}