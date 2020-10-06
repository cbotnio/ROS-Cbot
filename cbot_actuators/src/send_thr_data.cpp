/************************* Thruster node ***************************

-> Takes common mode and differential modes as inputs, peforms 
  desired conversion and send commands to thrusters

*******************************************************************/


#include "ros/ros.h"
#include "cbot_common/serial.hpp"
#include "cbot_ros_msgs/ThrusterData.h"

int thrusters_on;
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
    ros::param::get("thrusters_on",thrusters_on);
    if(thrusters_on)
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
    ros::init(argc, argv, "thruster_serial");
    ros::Time::init();
    ros::NodeHandle n;

    n.getParam("thrusters_on", thrusters_on);

    if(thrusters_on)
    {
        char thrusters_port[100];
        int thrusters_baud;
        std::string temp;
        n.getParam("thrusters_port", temp);
        strcpy(thrusters_port, temp.c_str());
        n.getParam("thrusters_baudrate", thrusters_baud);
        
        SERIAL hil(thrusters_port, thrusters_baud);
        fd_hil = hil.openNonCanonical(&oldtio, &newtio);
    }
    
    ros::Subscriber thr_sub = n.subscribe("/Thrusters", 5, thrusterCallback);
    ros::Timer timer = n.createTimer(ros::Duration(0.1),timerCallbck);
    ros::spin();
    return 0;
}