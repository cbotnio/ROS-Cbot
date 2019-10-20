#include "cbot_sensors/ahrs.hpp"
#include "cbot_sensors/gps.hpp"
#include "cbot_ros_msgs/SensorsStatus.h"
#include "cbot_common/serial.hpp"

#define MAX_S_PORTS 6
struct termios oldtio[MAX_S_PORTS], newtio[MAX_S_PORTS];
fd_set readfs;
struct timeval Timeout;
int result;
ros::Publisher gps_pub, ahrs_pub, sensor_status_pub;
double last_read_gps, last_read_ahrs;
char GPS_port[100], AHRS_port[100];
int GPS_baud, AHRS_baud;
int GPS_OK, AHRS_OK;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "SENSOR_NODE");
    ros::Time::init();
    ros::NodeHandle n;

    std::string temp;
    ros::param::getCached("GPS_port", temp);
    strcpy(GPS_port, temp.c_str());

    ros::param::getCached("AHRS_port", temp);
    strcpy(AHRS_port, temp.c_str());

    ros::param::getCached("GPS_baudrate", GPS_baud);
    ros::param::getCached("AHRS_baudrate", AHRS_baud);

    // GPS gps("/dev/ttyS81", B9600);
    GPS gps(GPS_port, GPS_baud);
    AHRS ahrs("/dev/ttyS83", B38400);
    // AHRS ahrs(AHRS_port, AHRS_baud);

    gps.openNonCanonical(&oldtio[0], &newtio[0]);
    ahrs.openCanonical(30, &oldtio[1], &newtio[1]); 

    last_read_gps = last_read_ahrs = ros::Time::now().toSec();

    sensor_status_pub = n.advertise<cbot_ros_msgs::SensorsStatus>("SENSOR", 1000);

    if(gps.fd > 0)
    {
        ROS_INFO("GPS port opened: %d", gps.fd);
        gps_pub = n.advertise<cbot_ros_msgs::GPS>("GPS", 1000);
    }
    else
    {
        ROS_ERROR("Couldn't open GPS port");
        exit(1);
    }

    if(ahrs.fd > 0)
    {
        ROS_INFO("AHRS port opened: %d", ahrs.fd);
        ahrs_pub = n.advertise<cbot_ros_msgs::AHRS>("AHRS", 1000);
        ahrs.initAhrs();
    }
    else
    {
        ROS_ERROR("Couldn't open AHRS port");
        exit(1);
    }

    while(1)
    {
        Timeout.tv_sec = 0;
        Timeout.tv_usec = 10000;

        cbot_ros_msgs::SensorsStatus sensor_status_msg;

        ahrs.writeAhrs();

        FD_SET(gps.fd, &readfs); 
        FD_SET(ahrs.fd, &readfs);

        result = select(FD_SETSIZE, &readfs, NULL, NULL, &Timeout);
        
        if (result > 0)
        {
            if (FD_ISSET(gps.fd, &readfs))
            { 
                cbot_ros_msgs::GPS temp;
                temp = gps.decode();
                gps_pub.publish(temp);
                last_read_gps = ros::Time::now().toSec();
                GPS_OK = 1;
            } 

            if (FD_ISSET(ahrs.fd, &readfs))
            {  
                cbot_ros_msgs::AHRS temp;
                temp = ahrs.decode();
                ahrs_pub.publish(temp);
                last_read_ahrs = ros::Time::now().toSec();
                AHRS_OK = 1;
            }
        }

        if((ros::Time::now().toSec() - last_read_gps) > 5)
        {
            ROS_ERROR("GPS FAIL");
            GPS_OK = 0;

        }

        if((ros::Time::now().toSec() - last_read_ahrs) > 5)
        { 
            ROS_ERROR("AHRS FAIL");
            AHRS_OK = 0;
        }

        sensor_status_msg.GPS_OK = GPS_OK;
        sensor_status_msg.AHRS_OK = AHRS_OK;

        sensor_status_pub.publish(sensor_status_msg);

    }

    return 0;
}