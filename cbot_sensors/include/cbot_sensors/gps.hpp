#ifndef GPS_H
#define GPS_H

#include "ros/ros.h"
#include "cbot_common/serial.hpp"
#include "cbot_ros_msgs/GPS.h"
#include "cbot_sensors/GPS/simple_gps.hpp"

class GPS: public SERIAL 
{
    public:
        GPS(const char *com_port_path_string, int BAUD_RATE, int type);
        ~GPS();
        void read_gps(cbot_ros_msgs::GPS& temp);
        int _type;

    private: 
    	SimpleGPS simple_gps;
};

#endif