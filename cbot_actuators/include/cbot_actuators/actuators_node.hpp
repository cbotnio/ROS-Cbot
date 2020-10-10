#ifndef ACTUATORS_NODE_H
#define ACTUATORS_NODE_H

#include "ros/ros.h"
#include "cbot_common/serial.hpp"
#include "cbot_ros_msgs/ThrusterInputs.h"
#include "cbot_ros_msgs/ThrusterCMDM.h"

//dynamic reconfiguration
#include <dynamic_reconfigure/server.h>
#include <cbot_actuators/ActuatorsConfig.h>

namespace cbot_actuators{
class ActuatorsNode{

	public:
		ActuatorsNode(const ros::NodeHandle nh,const ros::NodeHandle private_nh);
		~ActuatorsNode();

	private:
		ros::NodeHandle nh_;

		dynamic_reconfigure::Server<cbot_actuators::ActuatorsConfig> dyn_config_server_;

		ros::ServiceServer thruster_inputs_server;
		ros::ServiceServer thruster_cmdm_server;

		ros::Timer timer;

		struct termios oldtio, newtio;
		int fd_hil;
		char thrusters_port[100];
    	int thrusters_baud;

		int thrusters_on;
		double comm_mode_F, diff_mode_F;
		double comm_mode_V, diff_mode_V;
		float T1, T2, pre_T1, pre_T2, sat_F;
		float T3, T4, pre_T3, pre_T4, sat_V;

        void DynConfigCallback(cbot_actuators::ActuatorsConfig &config, uint32_t level);

		void calcT1T2(float comm_mode, float diff_mode, float *T11, float *T22, float *PreT11, float *PreT22, float thrust_saturation);
		
		bool thrusterInputsCallback(cbot_ros_msgs::ThrusterInputs::Request &req, cbot_ros_msgs::ThrusterInputs::Response &res);

		bool thrusterCMDMCallback(cbot_ros_msgs::ThrusterCMDM::Request &req, cbot_ros_msgs::ThrusterCMDM::Response &res);
		
		void timerCallbck(const ros::TimerEvent& event);

};	// End ActuatorsNode class

}	// End cbot_actuators namespace

#endif