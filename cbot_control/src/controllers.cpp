#include "cbot_control/controllers.hpp"

double CONTROLLERS::prev_yaw = 0.0;
double CONTROLLERS::prev_yawRate = 0.0;
double CONTROLLERS::yawAW = 0.0;
double CONTROLLERS::yawOut = 0.0;

double CONTROLLERS::prev_pitch = 0.0;
double CONTROLLERS::prev_pitchRate = 0.0;
double CONTROLLERS::pitchAW = 0.0;
double CONTROLLERS::pitchOut = 0.0;

double CONTROLLERS::err3Y = 0.0;
double CONTROLLERS::UY = 0.0;
double CONTROLLERS::int_depth_err = 0.0;
double CONTROLLERS::int_u_err = 0.0;

float CONTROLLERS::lqr_yaw(double desired_heading, double yaw, double yawRate, double Ts)
{
    double kyaw, kr, ki, kaw, yawSaturation;
    ros::param::getCached("yaw_lqr_kyaw",kyaw);
    ros::param::getCached("yaw_lqr_kr",kr);
    ros::param::getCached("yaw_lqr_ki",ki);
    ros::param::getCached("yaw_lqr_kaw",kaw);
    ros::param::getCached("yawSaturation",yawSaturation);
    double err1, err2, delta, prev_yaw, prev_yawRate, yawAW, yawOut;
    prev_yaw = CONTROLLERS::prev_yaw;  
    prev_yawRate = CONTROLLERS::prev_yawRate;
    yawAW = CONTROLLERS::yawAW;
    yawOut = CONTROLLERS::yawOut;

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

    Psidot = Psidot*kyaw;

    double Psidotdot = (yawRate - prev_yawRate)*kr;
    double feedback = (Psidot + Psidotdot)/Ts;
    CONTROLLERS::prev_yaw = yaw;
    CONTROLLERS::prev_yawRate = yawRate;
    err2 = ki*err1 - feedback;

    //Anti-windup
    double Udot = err2 - yawAW;
    CONTROLLERS::yawOut = yawOut + Udot*Ts;

    // Apply output saturations
    delta = CONTROLLERS::yawOut; 
    if (delta > yawSaturation)  delta = yawSaturation;
    if (delta < -yawSaturation) delta = -yawSaturation;

    //Update Anti windup
    CONTROLLERS::yawAW = kaw*(CONTROLLERS::yawOut - delta);

    return delta;
}

double CONTROLLERS::lqr_pitch(double desired_pitch, double pitch, double pitchRate, double Ts)
{
	double kpitch, kr, ki, kaw, pitchSaturation;
    double prev_pitch, prev_pitchRate, pitchAW, pitchOut;
    double err1, err2, delta;

    ros::param::getCached("pitch_lqr_kyaw",kpitch);
    ros::param::getCached("pitch_lqr_kr",kr);
    ros::param::getCached("pitch_lqr_ki",ki);
    ros::param::getCached("pitch_lqr_kaw",kaw);
    ros::param::getCached("pitchSaturation",pitchSaturation);

    prev_pitch = CONTROLLERS::prev_pitch;
    prev_pitchRate = CONTROLLERS::prev_pitchRate;
    pitchAW = CONTROLLERS::pitchAW;
    pitchOut = CONTROLLERS::pitchOut;

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

    Thetadot = Thetadot*kpitch;

    double Thetadotdot = (pitchRate - prev_pitchRate)*kr;
    double feedback = (Thetadot + Thetadotdot)/Ts;
    CONTROLLERS::prev_pitch = pitch;
    CONTROLLERS::prev_pitchRate = pitchRate;
    err2 = ki*err1 - feedback;

    //Anti-windup
    double Udot = err2 - pitchAW;
    CONTROLLERS::pitchOut = pitchOut + Udot*Ts;

    // Apply output saturations
    delta = CONTROLLERS::pitchOut; 
    if (delta > pitchSaturation)  delta = pitchSaturation;
    if (delta < -pitchSaturation) delta = -pitchSaturation;

    //Update Anti windup
    CONTROLLERS::pitchAW = kaw*(CONTROLLERS::pitchOut - delta);

    return delta;
}

double CONTROLLERS::depth(double desired_depth, double depth, double Ts)
{
	double Kp, Ki, depthSaturation ,err, depthCM;
	ros::param::getCached("depth_K",Kp);
    ros::param::getCached("depth_Ki",Ki);
    ros::param::getCached("depthSaturation",depthSaturation);
    err = desired_depth - depth;
    CONTROLLERS::int_depth_err = CONTROLLERS::int_depth_err + err*Ts;
    depthCM = Kp*err + Ki*CONTROLLERS::int_depth_err;

    if(fabs(depthCM) > depthSaturation){
        depthCM = depthSaturation*depthCM/fabs(depthCM);
        CONTROLLERS::int_depth_err = CONTROLLERS::int_depth_err - err*Ts;
    }

    printf("int_depth_err: %f | depthCM: %f\n",CONTROLLERS::int_depth_err,-depthCM);
    return -depthCM;
}

float CONTROLLERS::velocity(double desired_u,double u,double Ts){
    double err = desired_u - u;
    double yawcm = 40*err + 20*int_u_err;

    if(fabs(yawcm)<=60)
        int_u_err = int_u_err + err*Ts;
    else
        yawcm = yawcm/fabs(yawcm)*60;
    return yawcm;
}