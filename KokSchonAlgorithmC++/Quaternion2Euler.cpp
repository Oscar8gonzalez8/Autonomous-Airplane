#include "Quaternion2Euler.h"

void Quaternion2Euler(
	double* roll, 
	double* pitch, 
	double* yaw, 
	const double q[4]) 
{
	double e0 = q[0];
	double e1 = q[1];
	double e2 = q[2];
	double e3 = q[3];
	if (e0 < 0.0) {
		e0 = -e0; 
		e1 = -e1; 
		e2 = -e2; 
		e3 = -e3; 
	}
	*roll = atan2(2.0 * (e0 * e1 + e2 * e3), (e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2));
	*pitch = asin(max(-1.0, min(1.0, 2.0 * (e0 * e2 - e1 * e3))));
	*yaw = atan2(2.0 * (e0 * e3 + e1 * e2), (e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3));
}