/************************* Thruster node ***************************

-> Takes common mode and differential modes or individual thruster values as inputs, peforms 
  desired conversion and publish commands to thrusters

*******************************************************************/


#include "cbot_actuators/actuators_node.hpp"

namespace cbot_actuators{

ActuatorsNode::ActuatorsNode(const ros::NodeHandle nh,const ros::NodeHandle private_nh)
    :nh_(nh),
     dyn_config_server_(private_nh) 
{
	// Open Serial Port
    std::string temp;
    nh_.getParam("thrusters_port", temp);
    nh_.getParam("thrusters_baudrate", thrusters_baud);
    strcpy(thrusters_port, temp.c_str());

    SERIAL hil(thrusters_port, thrusters_baud);
    fd_hil = hil.openNonCanonical(&oldtio, &newtio);

    // Dynamic Parameter Reconfigure Server
    dynamic_reconfigure::Server<cbot_actuators::ActuatorsConfig>::CallbackType f;
    f = boost::bind(&ActuatorsNode::DynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    // Server to get thruster inputs
    thruster_cmdm_server = nh_.advertiseService("thruster_cmdm", &ActuatorsNode::thrusterCMDMCallback, this);
    thruster_inputs_server = nh_.advertiseService("thruster_inputs", &ActuatorsNode::thrusterInputsCallback, this);

    // Timer callback to write data on serial port
    timer = nh_.createTimer(ros::Duration(0.1),&ActuatorsNode::timerCallbck,this);
}

ActuatorsNode::~ActuatorsNode(){}

void ActuatorsNode::DynConfigCallback(cbot_actuators::ActuatorsConfig &config, uint32_t level)
{
    thrusters_on = config.thrusters_on;
    sat_F = config.saturation_f;
    sat_V = config.saturation_v;
}

// ASV so 2 thrusters
void ActuatorsNode::calcT1T2(float comm_mode, float diff_mode, float *T11, float *T22, float *PreT11, float *PreT22, float thrust_saturation)
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

bool ActuatorsNode::thrusterInputsCallback(cbot_ros_msgs::ThrusterInputs::Request &req, cbot_ros_msgs::ThrusterInputs::Response &res)
{
    res.T1 = T1 = req.T1;
    res.T2 = T2 = req.T2;
    res.T3 = T3 = req.T3;
    res.T4 = T4 = req.T4;

    return true;
}

bool ActuatorsNode::thrusterCMDMCallback(cbot_ros_msgs::ThrusterCMDM::Request &req, cbot_ros_msgs::ThrusterCMDM::Response &res)
{
    res.comm_mode_F = comm_mode_F = req.comm_mode_F;
    res.diff_mode_F = diff_mode_F = req.diff_mode_F;
    res.comm_mode_V = comm_mode_V = req.comm_mode_V;
    res.diff_mode_V = diff_mode_V = req.diff_mode_V;

    calcT1T2(comm_mode_F, diff_mode_F, &T1, &T2, &pre_T1, &pre_T2, sat_F);
    calcT1T2(comm_mode_V, diff_mode_V, &T3, &T4, &pre_T3, &pre_T4, sat_V);

    return true; 
}

void ActuatorsNode::timerCallbck(const ros::TimerEvent& event)
{
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

} // End cbot_actuators namespace

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "actuators_node");
    ros::Time::init();

    ros::NodeHandle nh, private_nh("~");

    cbot_actuators::ActuatorsNode actuator_node_(nh,private_nh);

    ros::spin();

    return 0;
}