#include <ros/ros.h>
#include <stdio.h>
#include <nav_msgs/Path.h>
#include "cbot_mission/utm.hpp"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

ros::Publisher waypoint_pub;
char Zone[20];

double wpt_list[6][2] = {{15.4507,73.8041},
					 {15.4515,73.8041},
					 {15.4515,73.8044},
					 {15.4507,73.8044},
					 {15.4507,73.8047},
					 {15.4515,73.8047}};

double UTME_ref, UTMN_ref, UTME, UTMN;
double latitude_ref, longitude_ref, latitude, longitude;

void addWaypoints(){
	nav_msgs::Path path;
	path.header.frame_id = "world";
	for(int i=0;i<6;i++){
		geometry_msgs::PoseStamped pose;
		UTM::LLtoUTM(23, wpt_list[i][0], wpt_list[i][1], &UTMN, &UTME, Zone);
		pose.pose.position.x = -UTME_ref + UTME;
		pose.pose.position.y = -UTMN_ref + UTMN;
		path.poses.push_back(pose);
	}
	waypoint_pub.publish(path);
}

int main(int argc, char *argv[]){
	ros::init(argc,argv,"waypoint_publisher");
	ros::NodeHandle n;

	// n.getParam("reference_latitude", latitude_ref);
 //    n.getParam("reference_longitude", longitude_ref);
    latitude_ref = 15.4507;
    longitude_ref = 73.8041;
    UTM::LLtoUTM(23, latitude_ref, longitude_ref, &UTMN_ref, &UTME_ref, Zone);

	waypoint_pub = n.advertise<nav_msgs::Path>("waypoint_publisher",1);

	ros::Rate r(1);
	while(ros::ok()){
		addWaypoints();
		r.sleep();
	}
}