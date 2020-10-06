#include "ros/ros.h"
#include "cbot_ros_msgs/GPS.h"
#include "geometry_msgs/Pose.h"
#include "cbot_navigation/utm.hpp"

double UTME_ref, UTMN_ref, UTME, UTMN;
double latitude_ref, longitude_ref, latitude, longitude;
double posx=0, posy=0;
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
    temp.position.y = UTME;// - UTME_ref;
    temp.position.x = UTMN;// - UTMN_ref;
    navigation_pub.publish(temp);

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navigation_node");
    ros::Time::init();
    ros::NodeHandle n;

    n.getParam("reference_latitude", latitude_ref);
    n.getParam("reference_longitude", longitude_ref);

    UTM::LLtoUTM(23, latitude_ref, longitude_ref, &UTMN_ref, &UTME_ref, Zone);
    navigation_pub = n.advertise<geometry_msgs::Pose>("/position", 1);
    ros::Subscriber gps_sub = n.subscribe("/GPS", 1, gpsCallback);
    ros::Timer timer = n.createTimer(ros::Duration(0.05), timerCallback);

    ros::spin();
}