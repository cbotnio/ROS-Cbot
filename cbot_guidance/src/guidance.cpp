/*
*********************************** CLASS *******************************
=> Contains WayPt, Linefollow, StationKeeping and Arcfollow guidance algorithms
*************************************************************************
*/

#include "cbot_guidance/guidance.hpp"

double GUIDANCE::acceptable_radius = 2;

double GUIDANCE::PsiC = 0;
double GUIDANCE::Kp_Thrust = 0.098; 
double GUIDANCE::Kd_Thrust = 0.0049;

double GUIDANCE::desired_pos_x1 = 0; 
double GUIDANCE::desired_pos_y1 = 0; 
double GUIDANCE::desired_pos_x2 = 0; 
double GUIDANCE::desired_pos_y2 = 0; 
double GUIDANCE::desired_pos_xc = 0; 
double GUIDANCE::desired_pos_yc = 0;
double GUIDANCE::desired_heading = 0;

double GUIDANCE::antiwindupLFW = 0; 
double GUIDANCE::last_d = 0;
double GUIDANCE::int_dist_err = 0;

double GUIDANCE::desiredThrustCMF=0;

int GUIDANCE::arc_follow_direction = 0;
double GUIDANCE::antiwindupARC = 0;

bool GUIDANCE::WayPtGuidance(double veh_x, double veh_y)
{
    double x_err, y_err, z_err, rho_squared, rho, vel;  // Calculate the desired heading using present position
    float eps = 0.0000001;

    x_err = desired_pos_x1 - veh_x;
    y_err = desired_pos_y1 - veh_y;

    if (fabs(y_err) < eps) y_err = eps;     // Divide-by-zero prevention
    
    desired_heading = atan2(y_err, x_err) * RAD_2_DEG;
    
    if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);

    rho_squared = x_err * x_err + y_err * y_err;
    rho = sqrt(rho_squared);
    if ((rho < acceptable_radius)) 
    {
        return true;
    }
    return false;
}


bool GUIDANCE::LineFollowGuidance(double veh_x, double veh_y, double Ts)
{
    double m, Beta, BetaN, EndPos, X1, X2, Y1, Y2, XEnd, YEnd, XNew, YNew, d, d_derivative, PsiDot, PsiH;
    float U = 0.5, KAUW = 1/Ts, PsiMax = 1, RefH;
    double x_err, y_err;
    float eps = 0.0000001;
    
    double x_1, y_1;

    X1 = GUIDANCE::desired_pos_x1;
    Y1 = GUIDANCE::desired_pos_y1;
    X2 = GUIDANCE::desired_pos_x2;
    Y2 = GUIDANCE::desired_pos_y2;

    // ROS_INFO_STREAM("Following line %f %f to %f %f",X1,Y1,X2,Y2);

    x_err = veh_x - X1;
    y_err = veh_y - Y1;
    
    y_1 = Y2 - Y1;
    x_1 = X2 - X1;

    BetaN = atan2(y_1, x_1+eps);

    XEnd = (X2-X1)*cos(-BetaN) - (Y2-Y1)*sin(-BetaN);

    XNew = x_err*cos(-BetaN) - y_err*sin(-BetaN);
    YNew = x_err*sin(-BetaN) + y_err*cos(-BetaN);

    d = -YNew;
    

    d_derivative = (d-last_d)/Ts;

    last_d = d;
    
    PsiDot = -(Kd_Thrust*d/U) - (d_derivative*Kp_Thrust/U) + GUIDANCE::antiwindupLFW;
    PsiC = GUIDANCE::PsiC + PsiDot*Ts;
    PsiH = GUIDANCE::PsiC;

    if (GUIDANCE::PsiC > PsiMax)
      PsiH = PsiMax;

    else if (GUIDANCE::PsiC < -PsiMax)
      PsiH = -PsiMax;

    antiwindupLFW = KAUW * (PsiH - GUIDANCE::PsiC);
    RefH = BetaN - asin(PsiH);

    desired_heading = RefH * RAD_2_DEG;
    
    if (XNew > (XEnd - 2)) 
        return true;

    return false;
}

// bool GUIDANCE::StKpGuidance(double veh_x, double veh_y)
// {
//     double x_err, y_err, z_err, dist_err;
//     float eps = 0.0000001;

//     x_err = GUIDANCE::desired_pos_x1 - veh_x;
//     y_err = GUIDANCE::desired_pos_y1 - veh_y;
//     dist_err = x_err*x_err + y_err*y_err - 1;
//     if (fabs(x_err) < eps) x_err += x_err/fabs(x_err)*eps;     // Divide-by-zero prevention

//     desired_heading = atan2(y_err, x_err) * RAD_2_DEG;
    
//     desiredThrustCMF = 0.5*asin(dist_err/(fabs(dist_err)+1))*2/PI;

//     return false;
// }

bool GUIDANCE::StKpGuidance(double veh_x, double veh_y)
{
    double x_err, y_err, z_err, dist_err;
    float eps = 0.0000001;

    x_err = GUIDANCE::desired_pos_x1 - veh_x;
    y_err = GUIDANCE::desired_pos_y1 - veh_y;
    dist_err = x_err*x_err + y_err*y_err - 1;
    if (fabs(x_err) < eps) x_err += x_err/fabs(x_err)*eps;     // Divide-by-zero prevention

    desired_heading = atan2(y_err, x_err) * RAD_2_DEG;
    
    desiredThrustCMF = 0.5*asin(dist_err/(fabs(dist_err)+1))*2/PI;

    return false;
}

bool GUIDANCE::ArcFollowGuidance(double veh_x, double veh_y) 
{
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

    double d_derivative, PsiDot, PsiH, KAUW = 1, PsiMax = 1, RefH, BetaN;
    MATRIX m1, m2, m3;
    m1.row = 2, m2.row = 2, m1.col = 2, m2.col = 1;
    m3.row = 2, m3.col = 1;

    if (arc_follow_direction)   // Dir=1 for counter clockwise
      k = -1;
    else
      k = 1;  

    Xs = GUIDANCE::desired_pos_x1;
    Ys = GUIDANCE::desired_pos_y1;
    Xe = GUIDANCE::desired_pos_x2;
    Ye = GUIDANCE::desired_pos_y2;
    Xc = GUIDANCE::desired_pos_xc;
    Yc = GUIDANCE::desired_pos_yc;

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

        BetaN = (PI / 2) - beta;

        d_derivative = (d - last_d);

        last_d = d;

        PsiDot = -(Kd_Thrust) * d - d_derivative * (Kp_Thrust) + antiwindupARC;
        PsiC = PsiC + PsiDot;
        PsiH = PsiC;

        if (PsiC > PsiMax)
            PsiH = PsiMax;
        if (PsiC < (-PsiMax))
            PsiH = -PsiMax;
        antiwindupARC = KAUW * (PsiH - PsiC);
        RefH = BetaN - asin(PsiH);

        desired_heading = RefH * RAD_2_DEG;
        if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);
    }
    return false;
}