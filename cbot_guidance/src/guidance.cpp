/*
*********************************** CLASS *******************************
=> Contains WayPt, Linefollow and Arcfollow guidance algorithms
*************************************************************************
*/

#include "cbot_guidance/guidance.hpp"

double GUIDANCE::acceptable_radius = 2;
double GUIDANCE::Kd_Thrust = 1; 
double GUIDANCE::Kp_Thrust = 1;

double GUIDANCE::desired_heading = 0;
double GUIDANCE::desired_pos_x1 = 0; 
double GUIDANCE::desired_pos_y1 = 0; 
double GUIDANCE::desired_pos_x2 = 0; 
double GUIDANCE::desired_pos_y2 = 0; 
double GUIDANCE::desired_pos_xc = 0; 
double GUIDANCE::desired_pos_yc = 0;
int GUIDANCE::arc_follow_direction = 0;
int GUIDANCE::firstcurrdiff = 0;
double GUIDANCE::antiwindup_curr = 0; 
double GUIDANCE::last_d = 0; 
double GUIDANCE::PsiC = 0;


bool GUIDANCE::WayPtGuidance(double veh_x, double veh_y)
{
    double x_err, y_err, z_err, rho_squared, rho, vel;  // Calculate the desired heading using present position
    float eps = 0.0000001;

    x_err = desired_pos_x1 - veh_x;
    y_err = desired_pos_y1 - veh_y;

    if (fabs(y_err) < eps) y_err = eps;     // Divide-by-zero prevention
    desired_heading = atan2(x_err, y_err) * RAD_2_DEG;
    if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);

    rho_squared = x_err * x_err + y_err * y_err;
    rho = sqrt(rho_squared);
    if (rho < acceptable_radius) 
    {
        return true;
    }
    return false;
}

bool GUIDANCE::LineFollowGuidance(double veh_x, double veh_y)
{
    float m, Beta, BetaN, EndPos, X1, X2, Y1, Y2, XEnd, YEnd, XNew, YNew, d, d_derivative, PsiDot, PsiH;
    float KAUW = 1, PsiMax = 1, RefH;
    float x_err, y_err;
    float eps = 0.0000001;
    float tempdist;

    double x_1, y_1;

    X1 = desired_pos_x1;
    Y1 = desired_pos_y1;
    X2 = desired_pos_x2;
    Y2 = desired_pos_y2;

    if(X1 != 0 and Y1 !=0 and X2 != 0 and Y2 != 0)
    {
        ROS_INFO("Following line %f %f to %f %f" , X1, Y1, X2, Y2);

        x_err = veh_x - X1;
        y_err = veh_y - Y1;
        tempdist = x_err * x_err + y_err * y_err;

        y_1 = Y2 - Y1;
        x_1 = X2 - X1;

        Beta = atan2(y_1, x_1);
        BetaN = (PI / 2) - Beta;

        XEnd = (X2 - X1) * cos(-Beta) - (Y2 - Y1) * sin(-Beta);

        XNew = x_err * cos(-Beta) - y_err * sin(-Beta);
        YNew = x_err * sin(-Beta) + y_err * cos(-Beta);

        d = YNew;
        if (firstcurrdiff == 1)
            d_derivative = (d - last_d);
        else
            d_derivative = d;

        last_d = d;
        firstcurrdiff = 1;

        PsiDot = -(Kd_Thrust) * d - d_derivative * Kp_Thrust + antiwindup_curr;
        PsiC = PsiC + PsiDot;
        PsiH = PsiC;

        if (PsiC > PsiMax)
          PsiH = PsiMax;

        if (PsiC < (-PsiMax))
          PsiH = -PsiMax;

        antiwindup_curr = KAUW * (PsiH - PsiC);
        RefH = BetaN - asin(PsiH);

        desired_heading = RefH * RAD_2_DEG;
        if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);

        if (XNew > (XEnd - 2)) 
        {
            firstcurrdiff = 0;
            return true;
        }
    }
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
        {
            firstcurrdiff = 0;
            return true;
        }

        m1.m[0][0] = cos(beta);
        m1.m[0][1] = sin(beta);
        m1.m[1][0] = -sin(beta);
        m1.m[1][1] = cos(beta);

        m2.m[0][0] = veh_x - Pdx;
        m2.m[1][0] = veh_y - Pdy;

        m3 = MATRIX::multiply(m1, m2);

        d = m3.m[1][0];

        BetaN = (PI / 2) - beta;

        if (firstcurrdiff == 1)
            d_derivative = (d - last_d);
        else
            d_derivative = d;

        last_d = d;
        firstcurrdiff = 1;

        PsiDot = -(Kd_Thrust) * d - d_derivative * (Kp_Thrust) + antiwindup_curr;
        PsiC = PsiC + PsiDot;
        PsiH = PsiC;

        if (PsiC > PsiMax)
            PsiH = PsiMax;
        if (PsiC < (-PsiMax))
            PsiH = -PsiMax;
        antiwindup_curr = KAUW * (PsiH - PsiC);
        RefH = BetaN - asin(PsiH);

        desired_heading = RefH * RAD_2_DEG;
        if (desired_heading < 0) desired_heading = 360 - fabs(desired_heading);
    }
    return false;
}