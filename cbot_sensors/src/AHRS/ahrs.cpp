#include "cbot_sensors/ahrs.hpp"


//*****************************************************************************
AHRS::AHRS(const char *com_port_path_string, int BAUD_RATE, int type) : SERIAL(com_port_path_string, BAUD_RATE)
{
    strcpy(com_port_path, com_port_path_string);
    BAUDRATE = BAUD_RATE;
    _type = type;
}

AHRS::~AHRS(){}

void AHRS::read_ahrs(cbot_ros_msgs::AHRS& temp)
{
	switch(_type){
		case 0: unsigned char buf[250];
				int res = read(fd,buf,250);
				vnav.read_ahrs_bin(temp,buf,res);
				break;
	}
}