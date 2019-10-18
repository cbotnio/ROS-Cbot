#ifndef _AHRS_H_
#define _AHRS_H_

#include "ros/ros.h"
#include "asv_ros_revamp/common/serial.hpp"
#include "asv_ros_revamp/AHRS.h"

#define AHRS_FAIL 0
#define AHRS_CALLIB 1
#define AHRS_GOOD 2

#define SYNC 0xFF


class AHRS: public SERIAL {
  public:
  	AHRS(const char *com_port_path_string, int BAUD_RATE);
    asv_ros_revamp::AHRS decode();
    void write_ahrs();
    void init_ahrs();
  private:
  	asv_ros_revamp::AHRS check_ahrs(unsigned char *temp, int res);
  	unsigned char checksum_ahrs(unsigned char *buf, int res);  
  	asv_ros_revamp::AHRS tokenize_AHRS_data(unsigned char *buf); 
  	float tofloat(unsigned char *buf);
};

#endif