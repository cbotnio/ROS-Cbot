#ifndef _GPS_H_
#define _GPS_H_

#include "ros/ros.h"
#include "asv_ros_revamp/common/serial.hpp"
#include "asv_ros_revamp/GPS.h"

#define GPS_FAIL  0
#define GPS_FIX_NO  1
#define GPS_FIX_YES 2

enum GPRMC_fields
{ 
  UtCTIME = 2, StAT = 3, LaT = 4, DiRN_LaT = 5, LoN = 6, DiRN_LoN = 7, SpD = 8, TrACK = 9, UtCDATE = 10, MaGVAR = 11, DiRN_MaG = 12
};

class GPS: public SERIAL {
  public:
  	GPS(const char *com_port_path_string, int BAUD_RATE);
    asv_ros_revamp::GPS decode();
    int gprmc_valid;
  private:
    unsigned char get_hex(char* value);
    int checksum(char* buf, int res, char*(chsum)); 
};

#endif