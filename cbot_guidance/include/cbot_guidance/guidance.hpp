#include <math.h>
#include "stdio.h"
#include "ros/ros.h"
#include "cbot_guidance/matrix.hpp"


#define PI 3.1415926535
#define RAD_2_DEG 180.0/PI

class GUIDANCE {
    public:
        static double desired_pos_x1, desired_pos_y1, desired_pos_x2, desired_pos_y2, desired_pos_xc, desired_pos_yc;
        
        // Waypoint Guidance
        static double desired_heading, acceptable_radius;
        static bool WayPtGuidance(double veh_x, double veh_y);
        
        //Line Follow Guidance
        static double last_d,PsiC,antiwindupLFW,Kd_Thrust, Kp_Thrust;
        static bool LineFollowGuidance(double veh_x, double veh_y, double Ts);

        //Station Keeping
        static double desiredThrustCMF, int_dist_err;
        static bool StKpGuidance(double veh_x, double veh_y);

        //Arc Following
        static int arc_follow_direction;
        static double antiwindupARC;
        static bool ArcFollowGuidance(double veh_x, double veh_y);
};