/*
*********************************** CLASS *******************************
=> Contains WayPt, Linefollow, StationKeeping and Arcfollow guidance algorithms
*************************************************************************
*/

#include "cbot_guidance/guidance.hpp"

namespace cbot_guidance{

Guidance::Guidance(const ros::NodeHandle& nh,const ros::NodeHandle& private_nh)
    :nh_(nh),
     private_nh_(private_nh),
     initialized_parameters_(false)
{
    initializeParameters();
}

Guidance::~Guidance(){}

void Guidance::initializeParameters(){
    capture_radius = 2.0;

    initialized_parameters_=true;
}

bool Guidance::WayPtGuidance(double veh_x, double veh_y)
{
    assert(initialized_parameters_==true);
    
    double x_err, y_err, z_err, rho_squared, rho, vel;  // Calculate the desired heading using present position
    float eps = 0.0000001;

    x_err = desired_pos_x1 - veh_x;
    y_err = desired_pos_y1 - veh_y;

    if (fabs(y_err) < eps) y_err = eps;     // Divide-by-zero prevention
    
    desired_heading = atan2(y_err, x_err) * RAD_2_DEG;
    
    if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);

    rho_squared = x_err * x_err + y_err * y_err;
    rho = sqrt(rho_squared);
    if ((rho < capture_radius)) 
    {
        return true;
    }
    return false;
}


bool Guidance::LineFollowGuidance(double veh_x, double veh_y, double Ts)
{
    assert(initialized_parameters_==true);

    double m, beta_n, x1, x2, y1, y2, x_end, y_end, x_new, y_new, d, d_derivative, psi_dot, psi_h;
    float u = 0.5, kauw = 1/Ts, psi_max = 1, ref_h;
    double x_err, y_err;
    float eps = 0.0000001;
    
    double x_1, y_1;

    x1 = desired_pos_x1;
    y1 = desired_pos_y1;
    x2 = desired_pos_x2;
    y2 = desired_pos_y2;

    // ROS_INFO_STREAM("Following line %f %f to %f %f",X1,Y1,X2,Y2);

    x_err = veh_x - x1;
    y_err = veh_y - y1;
    
    y_1 = y2 - y1;
    x_1 = x2 - x1;

    beta_n = atan2(y_1, x_1+eps);

    x_end = (x2-x1)*cos(-beta_n) - (y2-y1)*sin(-beta_n);

    x_new = x_err*cos(-beta_n) - y_err*sin(-beta_n);
    y_new = x_err*sin(-beta_n) + y_err*cos(-beta_n);

    d = -y_new;
    std::cout << "Distance from Line: " << d << std::endl;

    d_derivative = (d-last_d)/Ts;

    last_d = d;
    
    psi_dot = -(lfw_kd*d/u) - (d_derivative*lfw_kp/u) + lfw_antiwindup;
    lfw_psi_c = lfw_psi_c + psi_dot*Ts;
    psi_h = lfw_psi_c;

    if (lfw_psi_c > psi_max)
      psi_h = psi_max;

    else if (lfw_psi_c < -psi_max)
      psi_h = -psi_max;

    lfw_antiwindup = kauw * (psi_h - lfw_psi_c);
    ref_h = beta_n - asin(psi_h);

    desired_heading = ref_h * RAD_2_DEG;
    
    if (x_new > (x_end - capture_radius)) 
        return true;

    return false;
}

bool Guidance::StKpGuidance(double veh_x, double veh_y)
{
    assert(initialized_parameters_==true);

    double x_err, y_err, z_err, dist_err;
    float eps = 0.0000001;

    x_err = desired_pos_x1 - veh_x;
    y_err = desired_pos_y1 - veh_y;
    dist_err = x_err*x_err + y_err*y_err - 1;
    if (fabs(x_err) < eps) x_err += x_err/fabs(x_err)*eps;     // Divide-by-zero prevention

    desired_heading = atan2(y_err, x_err) * RAD_2_DEG;
    
    desired_thrust_cmf = 0.5*asin(dist_err/(fabs(dist_err)+1))*2/PI;

    return false;
}

bool Guidance::ArcFollowGuidance(double veh_x, double veh_y) 
{
    assert(initialized_parameters_==true);

    double gamma0 = 0.0, gammaend, gam, delta;
    short int k;
    double phi0 , phie, phi, beta;
    double Rad;
    double Xs, Xe, Xc, Ys, Ye, Yc;
    double x_err, y_err;
    int isin = 0, gotonext;
    double dPx, dPy, Pdx, Pdy;
    double dd, d;

    int firstArcFollow = 0;
    double heading;

    double d_derivative, psi_dot, psi_h, kauw = 1, psi_max = 1, ref_h, beta_n;
    MATRIX m1, m2, m3;
    m1.row = 2, m2.row = 2, m1.col = 2, m2.col = 1;
    m3.row = 2, m3.col = 1;

    if (arc_follow_direction)   // Dir=1 for counter clockwise
      k = -1;
    else
      k = 1;  

    Xs = desired_pos_x1;
    Ys = desired_pos_y1;
    Xe = desired_pos_x2;
    Ye = desired_pos_y2;
    Xc = desired_pos_xc;
    Yc = desired_pos_yc;

    if(Xs != 0 and Ys !=0 and Xe != 0 and Ye != 0)
    {

        x_err = Xs - Xc;
        y_err = Ys - Yc;

        Rad = sqrt(x_err * x_err + y_err * y_err);

        phi0 = atan2(Ys - Yc, Xs - Xc);
        phie = atan2(Ye - Yc, Xe - Xc);
        phi = atan2(veh_y - Yc, veh_x - Xc);

        if ( (k == 1 && (phie >= phi0) && (phi <= phie) && (phi >= phi0))
             || (k == 1 && (phie < phi0) && ((phi <= phie) || (phi >= phi0)))
             || (k == -1 && (phi0 >= phie) && (phi <= phi0) && (phi >= phie))
             || (k == -1 && (phi0 < phie) && ((phi <= phi0) || (phi >= phie)))
             || isin)
            isin = 1;
        else
            phi = phi0;

        dPx = -k * sin(phi);
        dPy = k * cos(phi);
        Pdx = Xc + Rad * cos(phi);
        Pdy = Yc + Rad * sin(phi);
        beta = atan2(dPy, dPx);
        gam = gamma0 + k * (phi - phi0) * Rad;
        
        if (gam < gamma0)
            gam = gam + 2 * PI * Rad;

        delta = k * (phie - phi0);
        if (delta < 0)
            delta = delta + 2 * PI;

        gammaend = gamma0 + Rad * delta;
        printf("\n gammaend::%f, gam::%f, k = %d", gammaend, gam, k);
        
        if (abs(gam - gammaend) < 3)
            return true;

        m1.m[0][0] = cos(beta);
        m1.m[0][1] = sin(beta);
        m1.m[1][0] = -sin(beta);
        m1.m[1][1] = cos(beta);

        m2.m[0][0] = veh_x - Pdx;
        m2.m[1][0] = veh_y - Pdy;

        m3 = MATRIX::multiply(m1, m2);

        d = m3.m[1][0];

        beta_n = (PI / 2) - beta;

        d_derivative = (d - last_d);

        last_d = d;

        psi_dot = -(arc_kd) * d - d_derivative * (arc_kp) + antiwindup_arc;
        arc_psi_c = arc_psi_c + psi_dot;
        psi_h = arc_psi_c;

        if (arc_psi_c > psi_max)
            psi_h = psi_max;
        if (arc_psi_c < (-psi_max))
            psi_h = -psi_max;
        antiwindup_arc = kauw * (psi_h - arc_psi_c);
        ref_h = beta_n - asin(psi_h);

        desired_heading = ref_h * RAD_2_DEG;
        if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);
    }
    return false;
}

}