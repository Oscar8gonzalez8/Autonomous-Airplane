#pragma once

#include <math.h>
#include <cmath>

#define max(a,b) ((a<b)?b:a)
#define min(a,b) ((a>b)?b:a)
using namespace std;

static double cosd(const double angleDeg);
static double sind(const double angleDeg);
// The Kok Schon Quaternion Estimator Algorithm 
extern int function_KokSchonQuaternionEstimator(
	double quaternion_new[4],
	const double quaternion[4],
	const double gyrometer[3],
	const double gravity[3],
	const double magnetometer[3],
	const double dt,
	const double inclinationAngleDegrees,
	const double alpha,
	const double beta
);