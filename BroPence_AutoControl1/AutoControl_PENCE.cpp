
#include "AutoControl_PENCE.h"
#include <math.h>
#include <iostream>
#include <cmath>

const double rEarth = 6371000;

static double cosd(double angleDegrees) {
    return cos(M_PI / 180.0 * angleDegrees);
}

static double norm3(double x[3]) {
    return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

static void convertGPS(
    double dXI[3],
    const double LatTarget,
    const double LonTarget,
    const double AltTarget,
    const double GPS_lat,
    const double GPS_lon,
    const double alt
) {
    dXI[0] = (rEarth * M_PI / 180.0) * (LatTarget - GPS_lat); // xI displacement
    double dLon = LonTarget - GPS_lon;
    if (fabs(dLon) > 180.0) {
        if (LonTarget > GPS_lon)
            dLon = LonTarget - (GPS_lon + 360.0);
        else
            dLon = (360.0 + LonTarget) - GPS_lon;
    }
    // yI displacement 
    dXI[1] = sign1(dLon) * rEarth * acos((cosd(dLon) - 1.0) * (cosd(GPS_lat) * cosd(GPS_lat)) + 1.0);
    // zI displacement
    dXI[2] = alt - AltTarget;
}

static void processWaypoints(
    double* pitchDes,
    double* yawDes,
    const double target_lat,
    const double target_lon,
    const double target_alt,
    const double GPS_lat,
    const double GPS_lon,
    const double alt
) {
    double dXI[3] = { 0.0 };
    // get the distance to the next waypoint
    convertGPS(dXI,
        target_lat,
        target_lon,
        target_alt,
        GPS_lat,
        GPS_lon,
        alt);
    double distance = norm3(dXI);

    *pitchDes = atan(-dXI[2] / fmax(1.0, distance));
    // get the desired yaw in rad
    *yawDes = atan2(dXI[1], dXI[0]);
}

static void Quaternion2Euler(
    double* roll,
    double* pitch,
    double* yaw,
    const double q[4]
) {
    double e0 = q[0];
    double e1 = q[1];
    double e2 = q[2];
    double e3 = q[3];
    *roll = atan2(2.0 * ((e0 * e1) + (e2 * e3)), e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2);
    *pitch = asin(fmax(-1.0, fmin(1.0, 2.0 * (e0 * e2 - e1 * e3))));
    *yaw = atan2(2.0 * ((e0 * e3) + (e1 * e2)), e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3);
}

static double getAngleError(double xd, double x) {
    double diff = xd - x;
    double d = acos(cos(xd) * cos(x) + sin(xd) * sin(x));

    if (fabs(diff) - d > 0.01) {
        return -sign1(diff) * d;
    }
    else {
        return diff;
    }
}

static void getCommands(
    double* delta_t,
    double* delta_e,
    double* delta_a,
    double* delta_r,
    const double roll,
    const double pitch,
    const double yaw,
    const double pitchDes,
    const double yawDes,
    const double gyro[3],
    const double kp_roll,
    const double kd_roll,
    const double kp_pitch,
    const double kp_yaw,
    const double kp_rudder,
    const double max_roll,
    const double max_pitch
) {
    double yaw_err = getAngleError(yawDes, yaw);
    double roll_des = kp_yaw * yaw_err;
    roll_des = fmax(-max_roll, fmin(max_roll, roll_des));

    double roll_err = getAngleError(roll_des, roll);

    double pitch_set = fmax(-max_pitch, fmin(pitchDes, max_pitch));
    double pitch_err = getAngleError(pitch_set, pitch);

    *delta_a = kp_roll * roll_err - kd_roll * gyro[0];
    *delta_e = kp_pitch * pitch_err;
    *delta_t = 0.8;
    *delta_r = kp_rudder * roll_err;

    *delta_a = fmax(-1.0, fmin(*delta_a, 1.0));
    *delta_e = fmax(-1.0, fmin(*delta_e, 1.0));
    *delta_r = fmax(-1.0, fmin(*delta_r, 1.0));
}

void AutoControl(
    double* delta_t,
    double* delta_e,
    double* delta_a,
    double* delta_r,
    // double* wpIndex,
    const double q[4],
    const double gyro[3],
    const double lat_target,
    const double lon_target,
    const double alt_target,
    // const double waypointIndex,
    // const double GPSwaypoints[5][3],
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
) {
    // get the estimated roll, pitch, and yaw
    double roll, pitch, yaw;
    Quaternion2Euler(&roll, &pitch, &yaw, q);

    // process the waypoints
    double pitchDes, yawDes;
    processWaypoints(&pitchDes, &yawDes, lat_target, lon_target, alt_target,
        latitue, longitude, altitude);

    getCommands(delta_t, delta_e, delta_a, delta_r, roll, pitch, yaw, pitchDes, yawDes, gyro,
        kp_roll, kd_roll, kp_pitch, kp_yaw, kp_rudder, max_roll, max_pitch);
}

double sign(double val) {
    if (val > 0) {
        return 1.0;
    }
    else if (val < 0) {
        return -1.0;
    }
    return 0.0;
}


