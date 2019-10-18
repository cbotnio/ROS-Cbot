#include "ros/ros.h"
#include "asv_ros_revamp/SENSOR.h"
#include "asv_ros_revamp/GPS.h"
#include "asv_ros_revamp/AHRS.h"

int GPS_status, AHRS_status;

void gps_callback(const asv_ros_revamp::GPS::ConstPtr& msg)
{
  GPS_status = msg->GPS_status;
}

void ahrs_callback(const asv_ros_revamp::AHRS::ConstPtr& msg)
{
  AHRS_status = msg->AHRS_Status;
}

void sensor_callback(const asv_ros_revamp::SENSOR::ConstPtr& msg)
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

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "SAFETY_NODE");
  ros::Time::init();
  ros::NodeHandle n;

  ros::Subscriber gps_sub = n.subscribe("GPS", 1000, gps_callback);  
  ros::Subscriber ahrs_sub = n.subscribe("AHRS", 1000, ahrs_callback);  
  ros::Subscriber sensor_sub = n.subscribe("SENSOR", 1000, sensor_callback);  

  ros::spin();

}