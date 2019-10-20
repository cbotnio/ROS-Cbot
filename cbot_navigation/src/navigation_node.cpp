#include "ros/ros.h"
#include "cbot_ros_msgs/GPS.h"
#include "geometry_msgs/Pose.h"
#include "cbot_navigation/utm.hpp"

double UTME_ref, UTMN_ref, UTME, UTMN;
double latitude_ref, longitude_ref, latitude, longitude;
char Zone[20];
ros::Publisher navigation_pub;

void gpsCallback(const cbot_ros_msgs::GPS::ConstPtr& msg)
{
    if(msg->GPS_status == 2)
    {
        latitude = msg->latitude;
        longitude = msg->longitude;
    }
}

void timerCallback(const ros::TimerEvent& event)
{
    UTM::LLtoUTM(23, latitude, longitude, &UTMN, &UTME, Zone);
    geometry_msgs::Pose temp;
    temp.position.x = UTME - UTME_ref;
    temp.position.y = UTMN - UTMN_ref;
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
    navigation_pub = n.advertise<geometry_msgs::Pose>("/position", 1000);
    ros::Subscriber gps_sub = n.subscribe("/GPS", 1000, gpsCallback);  
    ros::Timer timer = n.createTimer(ros::Duration(1), timerCallback);

    ros::spin();
}