#include "ros/ros.h"
#include "asv_ros_revamp/GPS.h"
#include "asv_ros_revamp/NAVIGATION.h"
#include "asv_ros_revamp/navigation/utm.hpp"

double UTME_ref, UTMN_ref, UTME, UTMN;
double latitude_ref, longitude_ref, latitude, longitude;
char Zone[20];
ros::Publisher navigation_pub;

void gps_callback(const asv_ros_revamp::GPS::ConstPtr& msg)
{
  if(msg->GPS_status == 2)
  {
    latitude = msg->latitude;
    longitude = msg->longitude;
  }
}

void timer_callback(const ros::TimerEvent& event)
{
	UTM::LLtoUTM(23, latitude, longitude, &UTMN, &UTME, Zone);
  asv_ros_revamp::NAVIGATION temp;
  temp.EST_POS_X = UTME - UTME_ref;
  temp.EST_POS_Y = UTMN - UTMN_ref;
  navigation_pub.publish(temp);

}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "NAVIGATION_NODE");
  ros::Time::init();
  ros::NodeHandle n;

  ros::param::getCached("reference_latitude", latitude_ref);
  ros::param::getCached("reference_longitude", longitude_ref);

  UTM::LLtoUTM(23, latitude_ref, longitude_ref, &UTMN_ref, &UTME_ref, Zone);
  navigation_pub = n.advertise<asv_ros_revamp::NAVIGATION>("NAVIGATION", 1000);
  ros::Subscriber gps_sub = n.subscribe("GPS", 1000, gps_callback);  
  ros::Timer timer = n.createTimer(ros::Duration(1), timer_callback);

  ros::spin();

}