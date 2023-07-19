#pragma once

#include <cmath>
#include <algorithm>
#define PI (3.14159265359f)

extern void AutoControl(
	double* delta_t,
	double* delta_e,
	double* delta_a,
	double* delta_r,
	double* wpIndex,
	const double q[4],
	const double gyro[3],
	const double waypointIndex,
	const double GPSwaypoints[5][3],
	const double GPS_lat,
	const double GPS_lon,
	const double alt,
	const double lat0,
	const double lon0,
	const double alt0
);

extern void Quaternion2Euler(double e0, double e1, double e2, double e3, double* roll, double* pitch, double* yaw);

extern double diff_in_angles(double alpha2, double alpha1);

extern double extract_yI(double lambda, double lam0, double phi);

extern double normalize(double delta_xI, double delta_yI, double delta_zI);

extern double trim_val(double variable, double limit);