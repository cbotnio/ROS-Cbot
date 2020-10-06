#include <cbot_control/controllers_node.hpp>

namespace cbot_control{

ControllersNode::ControllersNode(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     controllers_(nh,private_nh),
     dyn_config_server_(private_nh)
{
	// Dynamic reconfigure
    dynamic_reconfigure::Server<cbot_control::ControllersConfig>::CallbackType f;
    f = boost::bind(&ControllersNode::DynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    // Service Server to recieve control commands
    controller_inputs_server = nh_.advertiseService("controller_inputs", &ControllersNode::controllerInputsCallback,this);

    // Service Client to send thruster commands
    thruster_control_client = nh_.serviceClient<cbot_ros_msgs::ThrusterControl>("thruster_control");

    // Sensor subscribers
    ahrs_sub = nh_.subscribe("AHRS", 1, &ControllersNode::ahrsCallback, this);
    pose_sub = nh_.subscribe("/cbot/pose_gt", 1, &ControllersNode::poseCallback, this);

    // Loop for running the controller at 10Hz.
    timer = nh_.createTimer(ros::Duration(0.1), &ControllersNode::timerCallback, this);

}

ControllersNode::~ControllersNode(){}
    
void ControllersNode::DynConfigCallback(cbot_control::ControllersConfig &config,uint32_t level)
{
  Eigen::VectorXd k_yaw(4);
  Eigen::VectorXd k_pitch(4);
  Eigen::Vector2d k_depth;
  Eigen::Vector2d k_speed;

  k_yaw << config.yaw_k, config.yaw_kr, config.yaw_ki, config.yaw_kaw;
  k_pitch << config.pitch_k, config.pitch_kr, config.pitch_ki, config.pitch_kaw;
  k_depth << config.depth_k, config.depth_ki;
  k_speed << config.speed_k, config.speed_ki;

  controllers_.setYawGains(k_yaw);
  controllers_.setYawDMSaturation(config.yaw_dm_saturation);
  controllers_.setPitchGains(k_pitch);
  controllers_.setPitchDMSaturation(config.pitch_dm_saturation);
  controllers_.setDepthGains(k_depth);
  controllers_.setDepthCMSaturation(config.depth_cm_saturation);
  controllers_.setSpeedGains(k_speed);
  controllers_.setSpeedCMSaturation(config.speed_cm_saturation);

  heading_ctrl_on = (int)(config.heading_ctrl);
  pitch_ctrl_on = (int)(config.pitch_ctrl);
  depth_ctrl_on = (int)(config.depth_ctrl);
  speed_ctrl_on = (int)(config.speed_ctrl);
  controller_on = (int)(config.controller_on);
}

void ControllersNode::ahrsCallback(const cbot_ros_msgs::AHRS::ConstPtr& msg)
{
    if(msg->AHRS_Status == 2)
    {
        yaw = msg->YawAngle;
        yaw_rate = msg->YawRate;
        pitch = msg->Pitch;
        pitch_rate = msg->PitchRate;
    }
}

void ControllersNode::poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    depth = -msg->pose.pose.position.z;
    vx = msg->twist.twist.linear.y;
    vy = msg->twist.twist.linear.x;
    vz = -msg->twist.twist.linear.z;
}

double ControllersNode::getBodyVel()
{
    double p = pitch*deg2rad, y = yaw*deg2rad;
    return (cos(p)*cos(y)*vx + sin(y)*cos(p)*vy - sin(p)*vz);
}

bool ControllersNode::controllerInputsCallback(cbot_ros_msgs::ControllerInputs::Request &req, cbot_ros_msgs::ControllerInputs::Response &res)
{
    desired_heading = req.desired_heading;
    desired_thrust = req.desired_thrust;
    desired_pitch = req.desired_pitch;  
    desired_depth = req.desired_depth;
    desired_u = req.desired_u;
    res.desired_heading = desired_heading;
    res.desired_thrust = desired_thrust;
    res.desired_pitch = desired_pitch;
    res.desired_depth = desired_depth;
    res.desired_u = desired_u;
    return true;
}


void ControllersNode::timerCallback(const ros::TimerEvent& event)
{
	if(controller_on)
    {
        flag=0;
        double u = getBodyVel();
        // Heading Control
        if(heading_ctrl_on)
            differential_mode_F = controllers_.lqrYaw(desired_heading, yaw, yaw_rate, 0.1);
        else
            differential_mode_F = 0;

        // Pitch Control
        if(pitch_ctrl_on)
            differential_mode_V = controllers_.lqrPitch(desired_pitch, pitch, pitch_rate, 0.1);
        else
            differential_mode_V = 0;
        
        // Depth Control
        if(depth_ctrl_on)
            common_mode_V = controllers_.depth(desired_depth, depth, 0.1);
        else
            common_mode_V = 0;
        
        // Speed Control
        if(speed_ctrl_on)
            common_mode_F = controllers_.velocity(desired_u, u, 0.1);
        else
            common_mode_F = 0;

        // printf("u: %f | CF %f | DF %f | CV %f | DV %f\n",u, common_mode_F, differential_mode_F,common_mode_V, differential_mode_V);
    
    	// Call thruster service
        cbot_ros_msgs::ThrusterControl temp;
        temp.request.comm_mode_F = common_mode_F;
        temp.request.diff_mode_F = differential_mode_F;
        temp.request.comm_mode_V = common_mode_V;
        temp.request.diff_mode_V = differential_mode_V;
        temp.request.update = 1;
        
        if (ros::service::call("thruster_control", temp))
        {
        }
    }
    else if(!controller_on && flag==0){
        flag=1;
        cbot_ros_msgs::ThrusterControl temp;
        temp.request.comm_mode_F = 0;
        temp.request.diff_mode_F = 0;
        temp.request.comm_mode_V = 0;
        temp.request.diff_mode_V = 0;
        temp.request.update = 1;
    }
}

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "control_node");
    ros::Time::init();

    ros::NodeHandle nh, private_nh("~");
    cbot_control::ControllersNode cbot_controller_(nh,private_nh);
    
    ros::spin();
    
    return  0;
}