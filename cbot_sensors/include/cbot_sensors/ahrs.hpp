#ifndef _AHRS_H_
#define _AHRS_H_

#include "ros/ros.h"
#include <cbot_common/serial.hpp>
#include "cbot_ros_msgs/AHRS.h"

#define AHRS_FAIL 0
#define AHRS_CALLIB 1
#define AHRS_GOOD 2

#define SYNC 0xFF


class AHRS: public SERIAL {
  public:
      AHRS(const char *com_port_path_string, int BAUD_RATE);
      cbot_ros_msgs::AHRS decode();
      void writeAhrs();
      void initAhrs();
  private:
      cbot_ros_msgs::AHRS checkAhrs(unsigned char *temp, int res);
      unsigned char checksumAhrs(unsigned char *buf, int res);  
      cbot_ros_msgs::AHRS tokenizeAHRSData(unsigned char *buf); 
      float toFloat(unsigned char *buf);
};

#endif