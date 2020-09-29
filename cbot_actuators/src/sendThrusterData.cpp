/************************* Thruster node ***************************

-> Takes common mode and differential modes as inputs, peforms 
  desired conversion and send commands to thrusters

*******************************************************************/


#include "ros/ros.h"
#include "cbot_common/serial.hpp"
#include "cbot_ros_msgs/ThrusterData.h"

int HIL_ON;
struct termios oldtio, newtio;
int fd_hil;

float T1,T2,T3,T4;

void thrusterCallback(const cbot_ros_msgs::ThrusterData::ConstPtr& msg)
{
    T1 = msg->T1;
    T2 = msg->T2;
    T3 = msg->T3;
    T4 = msg->T4;
}

void timerCallbck(const ros::TimerEvent& event){
    ros::param::get("/HIL_ON",HIL_ON);
    if(HIL_ON)
    {
        char sbuf[200];
        int len = sprintf(sbuf, "Thr,%f,%f,%f,%f\r\n", T1, T2, T3, T4);
        write(fd_hil, sbuf, len);
    }
    else{
        T1=0;T2=0;T3=0;T4=0;
        char sbuf[200];
        int len = sprintf(sbuf, "Thr,0.0,0.0,0.0,0.0\r\n");
        write(fd_hil, sbuf, len);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "thrusterSerial");
    ros::Time::init();
    ros::NodeHandle n;

    n.getParam("HIL_ON", HIL_ON);

    if(HIL_ON)
    {
        char HIL_port[100];
        int HIL_baud;
        std::string temp;
        n.getParam("HIL_port", temp);
        strcpy(HIL_port, temp.c_str());
        n.getParam("HIL_baud", HIL_baud);
        
        SERIAL hil(HIL_port, HIL_baud);
        fd_hil = hil.openNonCanonical(&oldtio, &newtio);
    }
    // thruster_control_server = n.advertiseService("thruster_control", thrusterControlCallback);
    ros::Subscriber gps_sub = n.subscribe("/Thrusters", 5, thrusterCallback);
    ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallbck);
    ros::spin();
    return 0;
}