/************************* Thruster node ***************************

-> Takes common mode and differential modes as inputs, peforms 
  desired conversion and publish commands to thrusters

*******************************************************************/


#include "ros/ros.h"
#include "cbot_ros_msgs/ThrusterControl.h"
#include "cbot_ros_msgs/ThrusterData.h"


ros::ServiceServer thruster_control_server;
double comm_mode_F, diff_mode_F;
double comm_mode_V, diff_mode_V;
float T1=0, T2=0, PreT1, PreT2, sat_F = 100.0;
float T3=0, T4=0, PreT3, PreT4, sat_V = 100.0;

ros::Publisher thrusterPub;

// ASV so 2 thrusters
void calcT1T2(float comm_mode, float diff_mode, float *T11, float *T22, float *PreT11, float *PreT22, float thrust_saturation)
{

    *T11 = (comm_mode + diff_mode) / 2;
    *T22 = (comm_mode - diff_mode) / 2;

    // Saturation
    if (*T11 > thrust_saturation)  *T11 = thrust_saturation; 
    if (*T11 < (-thrust_saturation)) *T11 = -thrust_saturation;
    if (*T22 > thrust_saturation)  *T22 = thrust_saturation;
    if (*T22 < (-thrust_saturation)) *T22 = -thrust_saturation;

    // Rate Limiter
    // if (*T11 > (*PreT11 + 0.5)) { *T11 = *PreT11 + 0.5;  *PreT11 = *T11; }
    // else if (*T11 < (*PreT11 - 0.5)) { *T11 = *PreT11 - 0.5; *PreT11 = *T11; }
    // else  *PreT11 = *T11;

    // if (*T22 > (*PreT22 + 0.5)) { *T22 = *PreT22 + 0.5;  *PreT22 = *T22; }
    // else if (*T22 < (*PreT22 - 0.5)) { *T22 = *PreT22 - 0.5; *PreT22 = *T22; }
    // else  *PreT22 = *T22;

    // *T11 = *PreT11;
    // *T22 = *PreT22;
}

bool thrusterControlCallback(cbot_ros_msgs::ThrusterControl::Request &req, cbot_ros_msgs::ThrusterControl::Response &res)
{
    if(req.update)
    {
        comm_mode_F = req.comm_mode_F;
        diff_mode_F = req.diff_mode_F;
        comm_mode_V = req.comm_mode_V;
        diff_mode_V = req.diff_mode_V;

        calcT1T2(comm_mode_F, diff_mode_F, &T1, &T2, &PreT1, &PreT2, sat_F);
        calcT1T2(comm_mode_V, diff_mode_V, &T3, &T4, &PreT3, &PreT4, sat_V);

        cbot_ros_msgs::ThrusterData thrusterData;
        thrusterData.T1 = T1;
        thrusterData.T2 = T2;
        thrusterData.T3 = T3;
        thrusterData.T4 = T4;

        thrusterPub.publish(thrusterData);
    }
    
    res.comm_mode_F = comm_mode_F;
    res.diff_mode_F = diff_mode_F;
    res.comm_mode_V = comm_mode_V;
    res.diff_mode_V = diff_mode_V;
    
    return true;    
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "thruster_node");
    ros::Time::init();
    ros::NodeHandle n;

    thruster_control_server = n.advertiseService("thruster_control", thrusterControlCallback);
    thrusterPub = n.advertise<cbot_ros_msgs::ThrusterData>("/Thrusters",5);
    ros::spin();
    return 0;
}