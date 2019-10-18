#include "ros/ros.h"
#include "asv_ros_revamp/thruster_control.h"
#include "asv_ros_revamp/common/serial.hpp"


ros::ServiceServer thruster_control_server;
double comm_mode, diff_mode;
float T1, T2, PreT1, PreT2;
float thrust_saturation = 5;
int HIL_ON;
struct termios oldtio, newtio;
int fd_hil;

void calc_T1_T2(float comm_mode, float diff_mode, float *T1, float *T2)
{

  *T1 = (comm_mode + diff_mode) / 2;
  *T2 = (comm_mode - diff_mode) / 2;

  if (*T1 > thrust_saturation)  *T1 = 5; 
  if (*T1 < (-thrust_saturation)) *T1 = -5;
  if (*T2 > thrust_saturation)  *T2 = 5;
  if (*T2 < (-thrust_saturation)) *T2 = -5;


  if (*T1 > (PreT1 + 0.5)) { *T1 = PreT1 + 0.5;  PreT1 = *T1; }
  else if (*T1 < (PreT1 - 0.5)) { *T1 = PreT1 - 0.5; PreT1 = *T1; }
  else  PreT1 = *T1;

  if (*T2 > (PreT2 + 0.5)) { *T2 = PreT2 + 0.5;  PreT2 = *T2; }
  else if (*T2 < (PreT2 - 0.5)) { *T2 = PreT2 - 0.5; PreT2 = *T2; }
  else  PreT2 = *T2;

  *T1 = PreT1;
  *T2 = PreT2;
}

bool thruster_control_callback(asv_ros_revamp::thruster_control::Request &req, asv_ros_revamp::thruster_control::Response &res)
{
  if(req.update)
  {
    comm_mode = req.comm_mode;
    diff_mode = req.diff_mode;
    res.comm_mode = comm_mode;
    res.diff_mode = diff_mode;

    calc_T1_T2(comm_mode, diff_mode, &T1, &T2);
    if(HIL_ON)
    {
      char sbuf[100];
      int len = sprintf(sbuf, "Thr,%f,%f\r\n", comm_mode, diff_mode);
      write(fd_hil, sbuf, len);
    }
  }
  else
  {
    res.comm_mode = comm_mode;
    res.diff_mode = diff_mode;
  }

  return true;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "THRUSTER_NODE");
  ros::Time::init();
  ros::NodeHandle n;

  ros::param::getCached("HIL_ON", HIL_ON);

  if(HIL_ON)
  {
    char HIL_port[100];
    int HIL_baud;
    std::string temp;
    ros::param::getCached("HIL_port", temp);
    strcpy(HIL_port, temp.c_str());
    ros::param::getCached("HIL_baud", HIL_baud);
    
    SERIAL hil(HIL_port, HIL_baud);
    fd_hil = hil.open_non_canonical(&oldtio, &newtio);
  }

  thruster_control_server = n.advertiseService("thruster_control", thruster_control_callback);

  ros::spin();
  return 0;
}