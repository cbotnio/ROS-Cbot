#include "cbot_sensors/gps.hpp"

GPS::GPS(const char *com_port_path_string, int BAUD_RATE, int type) : SERIAL(com_port_path_string, BAUD_RATE)
{
    strcpy(com_port_path, com_port_path_string);
    BAUDRATE = BAUD_RATE;
    _type = type;
}

GPS::~GPS(){};

void GPS::read_gps(cbot_ros_msgs::GPS& temp)
{
    switch(_type){
        case 0: char buf[200];
                int res = read(fd,buf,150);
                simple_gps.read_gps(temp,buf,res);
                break;
    }
}