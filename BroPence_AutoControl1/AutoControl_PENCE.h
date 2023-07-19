#pragma once
#include<math.h>
#define PI (3.14159265359f)
#define max1(a,b) ((a<b)?a:b)
#define min1(a,b) ((a>b)?b:a)
#define sign1(a) ((a<0)? -1.0:1.0)

void AutoControl(
    double* delta_t,
    double* delta_e,
    double* delta_a,
    double* delta_r,
    //double* wpIndex,
    const double q[4],
    const double gyro[3],
    const double lat_target,
    const double lon_target,
    const double alt_target,
    //const double waypointIndex,
    //const double GPSwaypoints[5][3],
    const double latitue,
    const double longitude,
    const double altitude,
    const double kp_roll,
    const double kd_roll,
    const double kp_pitch,
    const double kp_yaw,
    const double kp_rudder,
    const double max_roll,
    const double max_pitch
    //const double lat0,
    //const double lon0,
    //const double alt0
);

//void GPS_WaypointFollowingFunction(
//    const double GPSwaypoints[5][3],
//    double waypointIndex,
//    /* double roll,
//    double pitch,
//    double yaw,*/
//    const double gyro[3],
//    double lat,
//    double lon,
//    double alt,
//    double kp_yaw,
//    double kp_roll,
//    double kd_roll,
//    double kp_roll_pitch,
//    double kp_pitch,
//    double pitch_y_trim,
//    double* delta_e,
//    double* delta_a,
//    double* wpIndex
//);

//double clip(double lower, double v, double upper);
//double shortest_angle(double a2, double a1);
//double get_yI1(double lambda, double lam0, double phi);
//extern double sine1(double a);
//extern double sqr1(double x);
