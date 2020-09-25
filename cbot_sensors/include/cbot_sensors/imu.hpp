#ifndef _IMU_AHRS_H_
#define _IMU_AHRS_H_

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
#include <ros/ros.h>

#define AHRS_FAIL 0
#define AHRS_CALLIB 1
#define AHRS_GOOD 2

#define SYNC 0xFF


typedef struct Nvsdata {
		uint64_t			timestart;
		float 				ucgX,ucgY,ucgZ;							//uncompansated gyro
		float 				temp,pressure;
		float				dX,dY,dZ;								//DeltaVel
		float				cgX,cgY,cgZ	;							//compansated gyro
		float 				yaw,pitch,roll;							//yaw pitch roll The attitude is given as a 3,2,1 Euler angle sequence describing the body frame with respect to the local North East Down (NED) frame.
		float 				accelN ,accelE,accelD;					//estimated accelaration (with gravity in NED frame)			//
		float 				laccelX,laccelY,laccelZ;				//linear accelaration(without gravity) 
		float 				laccelN,laccelE,laccelD;				//linear accelaration(without gravity in NED frame) 
		float 				yaw1,pitch1,roll1;						//The estimated attitude (Yaw, Pitch, Roll) uncertainty (1 Sigma), reported in degrees.
	} Nvsdata;

int gps_com;

class IMU_AHRS: public SERIAL {
	public:
		IMU_AHRS(const char *com_port_path_string, int BAUD_RATE);
		cbot_ros_msgs::AHRS read_ahrs_bin ( void );
	private:
		int checksum ( char *str );
		char *get_field ( char *str, int fld );
		unsigned short calculateCRC(unsigned char data[], unsigned int length);
		int calCRC ( unsigned char buf[] );
		cbot_ros_msgs::AHRS tokenizeAHRSData(Nvsdata *dptr); 
		float toFloat(unsigned char *buf);
};

#endif