#include "cbot_control/controllers.hpp"

namespace cbot_control {

Controllers::Controllers(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     initialized_parameters_(false)
{
    initializeParameters();
}

Controllers::~Controllers(){}

void Controllers::initializeParameters(){
    // Incase there are any others parameters to be added
    initialized_parameters_ = true;
}

double Controllers::lqrYaw(double desired_heading, double yaw, double yaw_rate, double Ts){

    assert(initialized_parameters_==true);

    double err1, err2, delta;

    //choose the smaller angle for rotation
    if (yaw < 0) yaw = 360 - fabs(yaw);
    err1 = (desired_heading - yaw);
    if (err1 > 180) 
        err1 = err1 - 360;
    else if (err1 < -180) 
        err1 = 360 + err1;

    double Psidot = yaw - prev_yaw;
    if (Psidot > 180) 
        Psidot = 360 - Psidot;
    else if (Psidot < -180) 
        Psidot = 360 + Psidot;

    Psidot = Psidot*yaw_k;

    double Psidotdot = (yaw_rate - prev_yaw_rate)*yaw_kr;
    double feedback = (Psidot + Psidotdot)/Ts;
    prev_yaw = yaw;
    prev_yaw_rate = yaw_rate;
    err2 = yaw_ki*err1 - feedback;

    //Anti-windup
    double udot = err2 - yaw_aw;
    yaw_out = yaw_out + udot*Ts;

    // Apply output saturations
    delta = yaw_out; 
    if (delta > yaw_dm_saturation)  delta = yaw_dm_saturation;
    else if (delta < -yaw_dm_saturation) delta = -yaw_dm_saturation;

    //Update Anti windup
    yaw_aw = yaw_kaw*(yaw_out - delta);

    return delta;
}

double Controllers::lqrPitch(double desired_pitch, double pitch, double pitch_rate, double Ts){

    assert(initialized_parameters_==true);

    double err1, err2, delta;

    //choose the smaller angle for rotation
    if (pitch < 0) pitch = 360 - fabs(pitch);
    err1 = desired_pitch - pitch;
    if (err1 > 180) 
        err1 = err1 - 360;
    else if (err1 < -180) 
        err1 = 360 + err1;
    
    double Thetadot = pitch - prev_pitch;
    if (Thetadot > 180)
        Thetadot =  360 - Thetadot;
    else if (Thetadot < -180)
        Thetadot = 360 + Thetadot;

    Thetadot = Thetadot*pitch_k;

    double Thetadotdot = (pitch_rate - prev_pitch_rate)*pitch_kr;
    double feedback = (Thetadot + Thetadotdot)/Ts;
    prev_pitch = pitch;
    prev_pitch_rate = pitch_rate;
    err2 = pitch_ki*err1 - feedback;

    //Anti-windup
    double udot = err2 - pitch_aw;
    pitch_out = pitch_out + udot*Ts;

    // Apply output saturations
    delta = pitch_out; 
    if (delta > pitch_dm_saturation)  delta = pitch_dm_saturation;
    if (delta < -pitch_dm_saturation) delta = -pitch_dm_saturation;

    //Update Anti windup
    pitch_aw = pitch_kaw*(pitch_out - delta);

    return delta;
}

double Controllers::depth(double desired_depth, double depth, double Ts){

    assert(initialized_parameters_==true);

    double err = desired_depth - depth;
    int_depth_err = int_depth_err + err*Ts;
    double depth_cm = depth_k*err + depth_ki*int_depth_err;

    if(fabs(depth_cm) > depth_cm_saturation){
        depth_cm = depth_cm_saturation*depth_cm/fabs(depth_cm);
        int_depth_err = int_depth_err - err*Ts;
    }

    return -depth_cm;
}

double Controllers::velocity(double desired_u,double u,double Ts){
    
    assert(initialized_parameters_==true);

    double err = desired_u - u;
    double yaw_cm = speed_k*err + speed_ki*int_u_err;

    if(fabs(yaw_cm)<=speed_cm_saturation)
        int_u_err = int_u_err + err*Ts;
    else
        yaw_cm = yaw_cm/fabs(yaw_cm)*speed_cm_saturation;
    return yaw_cm;
}

} // end cbot_control namespace