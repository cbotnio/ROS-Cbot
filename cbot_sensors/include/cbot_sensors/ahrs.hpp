#ifndef AHRS_H
#define AHRS_H

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>
#include <cbot_ros_msgs/AHRS.h>
#include <cbot_common/serial.hpp>
#include <cbot_sensors/AHRS/vnav.hpp>
#include <ros/ros.h>

#define AHRS_FAIL 0
#define AHRS_CALLIB 1
#define AHRS_GOOD 2

#define SYNC 0xFF

class AHRS: public SERIAL {
	public:
		AHRS(const char *com_port_path_string, int BAUD_RATE, int type);
		~AHRS();
		void read_ahrs(cbot_ros_msgs::AHRS& temp);
		int _type;
		
	private:
		VNAV vnav;
};

#endif