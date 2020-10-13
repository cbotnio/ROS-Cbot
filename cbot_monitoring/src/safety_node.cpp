//******************************************************
// Node to monitor any safety contraint voilation
//******************************************************

#include <cbot_monitoring/safety_node.hpp>

namespace cbot_monitoring{

SafetyNode::SafetyNode(const ros::NodeHandle nh, const ros::NodeHandle private_nh)
    :nh_(nh),
     dynamic_reconfigure_server_(private_nh)
{
    // Dynamic reconfigure
    dynamic_reconfigure::Server<cbot_monitoring::SafetyConfig>::CallbackType f;
    f = boost::bind(&SafetyNode::DynConfigCallback, this, _1, _2);
    dynamic_reconfigure_server_.setCallback(f);

    // Sensor subscribers
    gps_sub = nh_.subscribe("/GPS", 1, &SafetyNode::gpsCallback, this);
    ahrs_sub = nh_.subscribe("/AHRS", 1, &SafetyNode::ahrsCallback, this);
    sensor_sub = nh_.subscribe("/SENSOR", 1, &SafetyNode::sensorsStatusCallback, this);

    timer = nh_.createTimer(ros::Duration(0.1),&SafetyNode::timerCallback, this);
}

SafetyNode::~SafetyNode(){}

void SafetyNode::DynConfigCallback(cbot_monitoring::SafetyConfig &config, uint32_t level)
{
    max_roll = config.max_roll;
    max_pitch = config.max_pitch;
    max_speed = config.max_speed;
    max_depth = config.max_depth;
    safety_flag = config.safety_flag;
}

void SafetyNode::gpsCallback(const cbot_ros_msgs::GPS::ConstPtr& msg)
{
    gps_status = msg->GPS_status;
    speed = msg->vel;
}

void SafetyNode::ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg)
{
    roll = msg->Roll;
    pitch = msg->Pitch;
    yaw = msg->YawAngle;
    roll_rate = msg->RollRate;
    pitch_rate = msg->PitchRate;
    yaw_rate = msg->YawRate;
    ahrs_status = msg->AHRS_Status;

}

void SafetyNode::sensorsStatusCallback(const cbot_ros_msgs::SensorsStatus::ConstPtr& msg)
{
    if(msg->AHRS_OK)
    {
        if(!ahrs_status)
            ROS_WARN("[SAFETY NODE] AHRS FAIL");
    }
    else
    {
        ROS_WARN("[SAFETY NODE] AHRS FAIL");
    }

    if(msg->GPS_OK)
    {
        if(gps_status == 1)
            ROS_WARN("[SAFETY NODE] NO GPS FIX");
        else if (gps_status == 0)
            ROS_WARN("[SAFETY NODE] GPS FAIL 2");
    }
    else
    {
        ROS_WARN("[SAFETY NODE] GPS FAIL");
    }
}

bool SafetyNode::checkSafety(){
    if(fabs(roll) > max_roll){
        ROS_WARN("[SAFETY NODE] ROLL ANGLE REACHED LIMIT \nRAISED SAFETY FLAG");
        return true;
    }
    else if(fabs(pitch) > max_pitch){
        // ROS_WARN("[SAFETY NODE] PITCH ANGLE REACHED LIMIT \nRAISED SAFETY FLAG");
        return true;
    }
    else if(fabs(speed) > max_speed){
        ROS_WARN("[SAFETY NODE] VELOCITY REACHED LIMIT \nRAISED SAFETY FLAG");
        return true;
    }
    // else if(fabs(depth) > max_depth){
    //     ROS_WARN("[SAFETY NODE] DEPTH REACHED LIMIT \nRAISED SAFETY FLAG");
    //     return true;
    // }
    else{
        return false;
    }
}

void SafetyNode::timerCallback(const ros::TimerEvent& event){
    if(checkSafety() && safety_flag==0){
    	printf("Sendind safety flag true\n");
    	std_srvs::SetBool safety_srv;
    	safety_srv.request.data = true;
    	if(ros::service::call("/safety_trigger",safety_srv));
    	safety_flag = 1;
    }
    else if(!checkSafety() && safety_flag==1){
    	std_srvs::SetBool safety_srv;
    	printf("Sendind safety flag false\n");
    	safety_srv.request.data = false;
    	if(ros::service::call("/safety_trigger",safety_srv));
    	safety_flag = 0;
    }
}

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "safety_node");
    ros::Time::init();

    ros::NodeHandle nh, private_nh("~");

    cbot_monitoring::SafetyNode safety_node_(nh,private_nh);

    ros::spin();
}