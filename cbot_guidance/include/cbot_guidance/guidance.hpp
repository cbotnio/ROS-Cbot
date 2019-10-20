#include <math.h>
#include "stdio.h"
#include "ros/ros.h"
#include "cbot_guidance/matrix.hpp"


#define PI 3.1415926535
#define RAD_2_DEG 180.0/PI

class GUIDANCE {
    public:
        static bool WayPtGuidance(double veh_x, double veh_y);
        static bool LineFollowGuidance(double veh_x, double veh_y);
        static bool ArcFollowGuidance(double veh_x, double veh_y);
        static double desired_heading;
        static double desired_pos_x1, desired_pos_y1, desired_pos_x2, desired_pos_y2, desired_pos_xc, desired_pos_yc;
        static int arc_follow_direction;
        static double acceptable_radius;
        static int firstcurrdiff;
        static double antiwindup_curr, last_d, PsiC;
        static double Kd_Thrust, Kp_Thrust; 
};