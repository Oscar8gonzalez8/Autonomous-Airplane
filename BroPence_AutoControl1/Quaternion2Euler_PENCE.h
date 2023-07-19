#pragma once
#include <math.h>
#include <cmath>
#define max2(a,b) ((a<b)?b:a)
#define min2(a,b) ((a>b)?b:a)
void Quaternion2Euler(
	double* roll,
	double* pitch,
	double* yaw,
	const double q[4]);