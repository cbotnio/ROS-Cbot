#include <cbot_guidance/guidance_node.hpp>

namespace cbot_guidance{

GuidanceNode::GuidanceNode(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     guidance_(nh,private_nh),
     initialized_parameters_(false)
{
    initializeParameters();
}

GuidanceNode::~GuidanceNode(){}

void GuidanceNode::initializeParameters(){

    // Dynamic reconfigure
    dynamic_reconfigure::Server<cbot_guidance::GuidanceConfig>::CallbackType f;
    f = boost::bind(&GuidanceNode::DynConfigCallback, this, _1, _2);
    dyn_config_server_.setCallback(f);

    // guidance_mode = 0;
    guidance_inputs_server = nh_.advertiseService("guidance_inputs", &GuidanceNode::guidanceInputsCallback, this);
    controller_inputs_pub = nh_.advertise<cbot_ros_msgs::ControllerInputs>("controller_inputs",1);
    
    ahrs_sub = nh_.subscribe("/position", 1, &GuidanceNode::navigationCallback, this);
    guidance_status_pub = nh_.advertise<std_msgs::Bool>("/guidanceStatus", 1);
    timer = nh_.createTimer(ros::Duration(1), &GuidanceNode::timerCallback, this);

    initialized_parameters_=true;
}

void GuidanceNode::DynConfigCallback(cbot_guidance::GuidanceConfig &config, uint32_t level)
{
    guidance_.setLFWKp(config.lfw_kp);
    guidance_.setLFWKd(config.lfw_kd);
    guidance_.setARCKp(config.arc_kp);
    guidance_.setARCKd(config.arc_kd);
    guidance_on = config.guidance_on; 
}

void GuidanceNode::navigationCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
    vehicle_pos_x = msg->position.x;
    vehicle_pos_y = msg->position.y;
}

bool GuidanceNode::guidanceInputsCallback(cbot_ros_msgs::GuidanceInputs::Request &req, cbot_ros_msgs::GuidanceInputs::Response &res)
{
    printf("New Goal\n");
    
    guidance_mode = req.guidance_mode;
    printf("Guidancce Mode: %d\n",guidance_mode );
    UTM::LLtoUTM(23, req.desired_pos_x1, req.desired_pos_y1, &desired_pos_x1, &desired_pos_y1, Zone);
    UTM::LLtoUTM(23, req.desired_pos_x2, req.desired_pos_y2, &desired_pos_x2, &desired_pos_y2, Zone);
    
    guidance_.setDesiredX1(desired_pos_x1);
    guidance_.setDesiredY1(desired_pos_y1);
    guidance_.setDesiredX2(desired_pos_x2);
    guidance_.setDesiredY2(desired_pos_y2);
    guidance_.setDesiredXc(req.desired_pos_xc);
    guidance_.setDesiredYc(req.desired_pos_yc);
    guidance_.setArcDirection(req.arc_follow_direction);
    nominal_velocity = req.nominal_velocity;

    res.update_inputs = true;

    parameter_flag = 0;
}

void GuidanceNode::timerCallback(const ros::TimerEvent& event)
{
    assert(initialized_parameters_==true);
    
    if(guidance_on){
        flag=0;
        
        if(guidance_mode == 0){
            if(parameter_flag==0)
            {       
                dynamic_reconfigure::Config conf;
                int_param.name = "heading_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "pitch_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                int_param.name = "speed_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "depth_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                srv_req.config = conf;
                ros::service::call("/control_node/set_parameters", srv_req, srv_resp);
                parameter_flag==1;
            }
            
            guidance_status = guidance_.WayPtGuidance(vehicle_pos_x, vehicle_pos_y);
        }
        
        else if(guidance_mode == 1){
            if(parameter_flag==0)
            {   
                dynamic_reconfigure::Config conf;
                int_param.name = "heading_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "pitch_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                int_param.name = "speed_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "depth_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                srv_req.config = conf;
                ros::service::call("/control_node/set_parameters", srv_req, srv_resp);
                parameter_flag=1;
            }
            guidance_status = guidance_.LineFollowGuidance(vehicle_pos_x, vehicle_pos_y, 1);
        }
        
        else if(guidance_mode == 2)
            guidance_status = guidance_.ArcFollowGuidance(vehicle_pos_x, vehicle_pos_y);  // To be completed
        
        else if(guidance_mode == 3){
            if(parameter_flag==0)
            {
                dynamic_reconfigure::Config conf;
                int_param.name = "heading_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "pitch_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                int_param.name = "speed_ctrl"; int_param.value = 1; conf.ints.push_back(int_param);
                int_param.name = "depth_ctrl"; int_param.value = 0; conf.ints.push_back(int_param);
                srv_req.config = conf;
                ros::service::call("/control_node/set_parameters",srv_req, srv_resp);
                parameter_flag = 1;
            }
            
            guidance_status = guidance_.StKpGuidance(vehicle_pos_x, vehicle_pos_y);
            nominal_velocity = guidance_.getDesiredSpeed();
        }

        if(guidance_status){
            nominal_velocity = 0;
        }
        
        std_msgs::Bool reached;
        reached.data = (guidance_status)?true:false;
        guidance_status_pub.publish(reached);

        cbot_ros_msgs::ControllerInputs control_inputs;
        control_inputs.desired_heading = guidance_.getDesiredHeading();
        control_inputs.desired_u = nominal_velocity;
        control_inputs.desired_pitch = 0;
        control_inputs.desired_depth = 1;
        
        controller_inputs_pub.publish(control_inputs);
    }

    else if(!guidance_on && flag==0){
        flag=1;
        cbot_ros_msgs::ControllerInputs control_inputs;
        control_inputs.desired_heading = guidance_.getDesiredHeading();
        control_inputs.desired_u = 0;
        
        controller_inputs_pub.publish(control_inputs);
    }
}

} // end of cbot_guidance namespace


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "guidance_node");
    ros::Time::init();

    ros::NodeHandle nh, private_nh("~");

    cbot_guidance::GuidanceNode cbot_guidance_(nh, private_nh);

    ros::spin();
    return  0;
}
