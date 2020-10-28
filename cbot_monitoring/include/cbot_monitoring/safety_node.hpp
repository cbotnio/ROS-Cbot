#ifndef SAFETY_NODE_H
#define SAFETY_NODE_H

#include "ros/ros.h"
#include "cbot_ros_msgs/SensorsStatus.h"
#include "cbot_ros_msgs/GPS.h"
#include "cbot_ros_msgs/AHRS.h"
#include "std_srvs/SetBool.h"

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <cbot_monitoring/SafetyConfig.h>

namespace cbot_monitoring{

class SafetyNode{
	public:
		SafetyNode(const ros::NodeHandle nh, const ros::NodeHandle private_nh);
		~SafetyNode();

	private:

		ros::NodeHandle nh_;

		ros::Subscriber gps_sub, ahrs_sub, sensor_sub;
	    ros::Timer timer;

		double max_roll, max_pitch, max_speed, max_depth;
		double roll, roll_rate, pitch, pitch_rate, yaw, yaw_rate;
		double speed, depth;
		int ahrs_status, gps_status, safety_flag;

		dynamic_reconfigure::Server<cbot_monitoring::SafetyConfig> dynamic_reconfigure_server_;

		void DynConfigCallback(cbot_monitoring::SafetyConfig &config, uint32_t level);
		void gpsCallback(const cbot_ros_msgs::GPS::ConstPtr& msg);
		void ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg);
		void sensorsStatusCallback(const cbot_ros_msgs::SensorsStatus::ConstPtr& msg);
		bool checkSafety();
		void timerCallback(const ros::TimerEvent& event);
};

}

#endif