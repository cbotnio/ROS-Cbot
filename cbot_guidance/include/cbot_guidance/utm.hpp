#ifndef UTM_H
#define UTM_H

#include "ros/ros.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PI    3.1415926535
#define FOURTHPI  PI/4.0
#define deg2rad   PI/180.0
#define rad2deg   180.0/PI

typedef struct elipse
{
    int id;
    char ellipsoidName[80];
    double EquatorialRadius; 
    double eccentricitySquared;  
} Ellipsoid;


class UTM 
{
    public:
        static void LLtoUTM(int ReferenceEllipsoid, const double Lat, const double Long, double *UTMNorthing, double *UTMEasting, char* UTMZone);
        static void UTMtoLL(int ReferenceEllipsoid, const double UTMNorthing, const double UTMEasting, const char* UTMZone,  double* Lat,  double* Long );
        static char UTMLetterDesignator(double Lat);
};

#endif