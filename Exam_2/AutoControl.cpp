#include "AutoControl.h"
#include <math.h>

using namespace std;

const double rEarth = 6371000; // // Radius of the Earth in meters (constant) 

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
)
{
	// Origin
	// Already given???
	//double lat0 = GPSwaypoints[0][0];
	//double lon0 = GPSwaypoints[0][1];
	//double alt0 = GPSwaypoints[0][2];

	// Desired waypoints
	int index = static_cast<int>(waypointIndex); // Convert waypointIndex to integer
	double lat_des = GPSwaypoints[index][0]; // Latitude of desired waypoint
	double lond_des = GPSwaypoints[index][1]; // Longitude of desired waypoint
	double altd = GPSwaypoints[index][2]; // Altitude of desired waypoint

	//Get the commands
	//*delta_t = 0.8;
	//*delta_e = -0.1;
	//*delta_a = -0.2;
	//*delta_r = 0.0;
	//*wpIndex = 0.0;

	double e0 = q[0];
	double e1 = q[1];
	double e2 = q[2];
	double e3 = q[3];


	// Convert quaternions to Euler angles (roll, pitch, yaw)
	double roll, pitch, yaw;
	Quaternion2Euler(e0, e1, e2, e3, &roll, &pitch, &yaw);

	// Convert latitude and longitude to radians
	double phi = GPS_lat * PI / 180.0;
	double lambda = GPS_lon * PI / 180.0;
	double phi0 = lat0 * PI / 180.0;
	double lam0 = lon0 * PI / 180.0;
	double latd = lat_des * PI / 180.0;
	double lond = lond_des * PI / 180.0;

	// Calculate inertial displacements of current position from the origin
	double xI = rEarth * (phi - phi0);
	double yI = extract_yI(lambda, lam0, phi);
	double zI = -(alt - alt0);

	// Calculate inertial displacements of desired waypoint from the origin
	double xId = rEarth * (latd - phi0);
	double yId = extract_yI(lond, lam0, latd);
	double zId = -(altd - alt0);

	// Calculate differences in inertial displacements
	double delta_xI = xId - xI;
	double delta_yI = yId - yI;
	double delta_zI = zId - zI;

	// Calculate the absolute distance to the waypoint
	double absolute_dist = normalize(delta_xI, delta_yI, delta_zI);

	// If the distance is less than 25 meters, move to the next waypoint
	if (absolute_dist < 25.0) {
		*wpIndex = waypointIndex + 1.0;
	}
	else {
		*wpIndex = waypointIndex;
	}

	// Given output values from instructor  
	
	*delta_t = 1.0;

	/*
	// PID controller gains for roll
	double kp_roll_yaw = 0.39;
	double kp_roll = 0.61;
	double kd_roll = 0.0684;
	double kp_delta_r = 0.5;
	double kp_pitch = -0.9;
	double kp_roll_pitch = 0.09;
	*/


	// PID controller gains for roll
	double kp_roll_yaw = 0.5;
	double kp_roll = 5;
	double kd_roll = 0.0684;
	double kp_delta_r = 0.5;
	double kp_pitch = -0.9;
	double kp_roll_pitch = 0.09;	

	// Calculate desired yaw using the difference in x and y inertial displacements
	double yaw_desired = atan2(delta_yI, delta_xI);

	// Calculate yaw error
	double u5 = diff_in_angles(yaw_desired, yaw);

	// Control input for roll based on yaw error
	double u4 = kp_roll_yaw * u5;

	// Calculate the reference roll, limiting it to [-PI/4, PI/4]
	double roll_r = trim_val(u4, PI / 11.0);

	// Calculate roll error.
	double u3 = diff_in_angles(roll_r, roll);

	// Calculate roll control input using PD controller.
	double u2 = kp_roll * u3;

	double u1 = u2 - (kd_roll * gyro[1]);

	// Update delta_a (aileron) to control roll.
	*delta_a = trim_val(u1, 1.0);



	double pitch_desired = asin(-delta_zI / absolute_dist);
	double theta_y_trim = 0.0;


	double theta_y_r = pitch_desired + theta_y_trim + kp_roll_pitch * u3 * u3;

	double u6 = diff_in_angles(theta_y_r, pitch);

	double u7 = kp_pitch * u6;

	*delta_e = trim_val(u7, 1.0);

	// u3 is the roll error 
	// implement rudder control 
	double deltar = u3 * kp_delta_r; 
	*delta_r = trim_val(deltar, 1);
}

// Quaternion2Euler converts the orientation of the aircraft from quaternion representation to Euler angles.
// Parameters:
// - e0, e1, e2, e3: quaternion components
// - roll, pitch, yaw: pointers to store the resulting Euler angles (in radians)
extern void Quaternion2Euler(double e0, double e1, double e2, double e3, double* roll, double* pitch, double* yaw) {
	// Compute roll angle and store in *roll
	if (roll) {
		*roll = atan2(2.0 * (e0 * e1 + e2 * e3), e0 * e0 + e3 * e3 - e1 * e1 - e2 * e2);
	}

	// Compute pitch angle and store in *pitch, making sure asin argument is within [-1, 1]
	if (pitch) {
		*pitch = asin(max(-1.0, min(1.0, 2.0 * (e0 * e2 - e1 * e3))));
	}

	// Compute yaw angle and store in *yaw
	if (yaw) {
		*yaw = atan2(2.0 * (e0 * e3 + e1 * e2), e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3);
	}
}


// Calculates the difference between two angles (in radians) taking into account the circular nature of angles.
extern double diff_in_angles(double alpha2, double alpha1) {
	double delta = acos(cos(alpha1) * cos(alpha2) + sin(alpha1) * sin(alpha2));
	delta = abs(delta);

	// Check for angle wraparound
	if (abs(alpha2 - alpha1) - delta > 0.1) {
		return -copysign(abs(delta), alpha2 - alpha1);
	}
	else {
		return alpha2 - alpha1;
	}
}

// Calculate the y component of inertial position (in the Earth frame) from longitude, reference longitude, and latitude (in radians).
extern double extract_yI(double lambda, double lam0, double phi) {
	double dlambda;

	// Handle longitude wrap-around (ensuring that dlambda is in the range [-PI, PI])
	if (abs(lambda - lam0) <= PI) {
		dlambda = lambda - lam0;
	}
	else {
		if (lambda > lam0) {
			dlambda = lambda - (lam0 + 2 * PI);
		}
		else {
			dlambda = (2 * PI + lambda) - lam0;
		}
	}

	// Calculate yI using the great circle distance formula
	double yI = (dlambda > 0 ? 1 : -1) * rEarth * acos((cos(dlambda) - 1) * pow(cos(phi), 2) + 1);
	return yI;
}

// Normalize a 3D vector by computing its Euclidean norm (magnitude).
extern double normalize(double delta_xI, double delta_yI, double delta_zI) {
	return sqrt(delta_xI * delta_xI + delta_yI * delta_yI + delta_zI * delta_zI);
}

// Trim a variable to be within limits (-limit to +limit).
extern double trim_val(double variable, double limit) {
	return max(-limit, min(variable, limit));
}

