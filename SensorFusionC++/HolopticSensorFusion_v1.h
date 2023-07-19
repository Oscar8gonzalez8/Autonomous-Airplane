#pragma once

#include <math.h>
#include <cmath>

#define PI (3.14159265359f)
#define max(a,b) ((a<b)?b:a)
#define min(a,b) ((a>b)?b:a)


//Create a cross-product matrix
extern void CrossMatrix(
	double CrossMat[3][3], //OUTPUT: Cross product matrix
	const double vec3[3] //INPUT: vector to be converted to a cross product matrix
);


//Inertial to body rotation matrix
extern void Rotation_I2b(
	double Ri2b[3][3],  //OUTPUT: Rotation matrix from the inertial to body frame (Rows first, then columns)
	const double e0, //INPUT: scalar part of the quaternion orientation in the inertial frame
	const double e1, //INPUT: x-axis part of the quaternion orientation in the inertial frame
	const double e2, //INPUT: y-axis part of the quaternion orientation in the inertial frame
	const double e3  //INPUT: z-axis part of the quaternion orientation in the inertial frame
);

//Rotate a vec3 from the inertial frame to the body frame
//extern void inertial_2_body_Rotation(
//	double* x_b, //OUTPUT: x-position in the body frame
//	double* y_b, //OUTPUT: y-position in the body frame
//	double* z_b, //OUTPUT: z-position in the body frame
//	const double xI, //INPUT: x-position in the inertial frame
//	const double yI, //INPUT: y-position in the inertial frame
//	const double zI, //INPUT: z-position in the inertial frame
//	const double e0, //INPUT: scalar part of the quaternion orientation in the inertial frame
//	const double e1, //INPUT: x-axis part of the quaternion orientation in the inertial frame
//	const double e2, //INPUT: y-axis part of the quaternion orientation in the inertial frame
//	const double e3  //INPUT: z-axis part of the quaternion orientation in the inertial frame
//);


//Normalize a 4d array
extern void vec4_norm(
	double C[4], //OUTPUT: normalized 4d array
	const double a[4] //INPUT: a 4d array to be normalized
);

//Normalize a 3d array
extern void vec3_norm(
	double C[3], //OUTPUT: normalized 3d array
	const double a[3] //INPUT: a 3d array to be normalized
);

//Matrix vector multiplication c = A*b
extern void Matrix3_Vec3_Multiplication(
	double c[3], //OUTPUT: output vector c = A*b
	const double A[3][3], //INPUT: input matrix c = A*b
	const double b[3] //INPUT: input vector c = A*b
);

//Multiply a vector by a scalar c = b * a
extern void Vec3_Scalar_Multiplication(
	double c[3], //OUTPUT: output vector c = b * a
	const double b[3], //INPUT: input vector c = b*a
	const double a //INPUT: scalar multiplier c = b*a
);

//Calculate the cross product between two 3d arrays
extern void vec3_cross(
	double C[3], //The cross product C = aXb
	const double a[3], //The 3D vector a in C=aXb
	const double b[3] //The 3D vector b in C=aXb
);

//Add two 3d vectors c=a+b
extern void vec3_addition(
	double c[3], //OUTPUT: output vector c = a + b
	const double a[3], //INPUT: input vector c = a + b
	const double b[3] //INPUT: input vector c = a + b
);

//subtract one 3d vector from another c=a-b
extern void vec3_subtraction(
	double c[3], //OUTPUT: output vector c = a - b 
	const double a[3], //INPUT: input vector c = a - b
	const double b[3] //INPUT: input vector c = a - b
);

//Calculate the gradient of the acceleration and magnetometer cost function V
//see Manon Kok and Thomas B. Schon, ``A Fast and Robust Algorithm for Orientation Estimation
// using Inertial Sensors'', IEEE Signal Processing Letters, 2019. DOI: 10.1109 / LSP.2019.2943995
extern void KokSchonGradient(
	double gradient[3],
	const double NormalizedAccelerometer[3], //aka g_ak accelerometer's estimate of gravity
	const double NormalizedMagnetometer[3], // aka 'm' Magnetometer's measurement of geomagnetic vector 
	const double mGyro[3], // Gyrometer's estimate of the geomagnetic vector
	const double gGyro[3], // Gyrometer's estimate of gravity
	const double alpha);

extern void KokSchonUpdate(
	double quaternion_new[4], //OUTPUT: updated quaternion orientation
	const double quaternion[4], //INPUT: previous quaternion orientation
	const double gradient[3], //INPUT: Kok Schon Gradient vector
	const double gyrometer[3], //INPUT: [rad/s] 3-axis gyro data with bias removed
	const double dt, //INPUT: [s] time-step
	const double beta //INPUT: [<< 1] small-valued tuning parameter 
);

//Run the Holoptic Sensor Fusion Algorithm
extern int runHolopticSensorFusion(
	double quaternion_new[4], //OUTPUT: updated quaternion orientation
	double velocities[3], //OUTPUT: [m/s] updated body-frame velocities estimate
	double GPS_velocities[3], //OUTPUT: [m/s] updated inertial-frame velocities estimate
	double GPS_positions[3], //OUTPUT: [m] updated inertial positions estimate
	double* t_GPS_new, //OUTPUT: [s] the most recent timestamp for new GPS data
	const double quaternion[4], //INPUT: previous quaternion orientation
	const double GPS_velocities_old[3], //INPUT: [m/s] previous estimate of inertial-frame velocities
	const double GPS_positions_old[3], //INPUT: [m] previous estimate of inertial-frame positions
	const double gyrometer[3], //INPUT: [rad/s] 3-axis gyro data with bias removed
	const double accelerometer[3], //INPUT: [m/s2] x-front, y-right, z-down raw accelerometer signal, bias removed
	const double magnetometer[3], //INPUT: [any] x-front, y-right, z-down calibrated magnetometer signal any units, bias removed
	const double GPS_latitude, //INPUT: [degrees] GPS latitude
	const double GPS_longitude, //INPUT: [degrees] GPS longitude
	const double altitude, //INPUT: [m] GPS altitude or barometric pressure altitude
	const double GPS_latitude_origin, //INPUT: [degrees] latitude at the origin [0,0,0]
	const double GPS_longitude_origin, //INPUT: [degrees] longitude at the origin [0,0,0]
	const double altitude_origin, //INPUT: [m] altitude at the origin [0,0,0]
	const double t_now, //INPUT: [s] present run-time (time right now)
	const double t_GPS, //INPUT: [s] last time new GPS data arrived
	const double dt, //INPUT: [s] time-step
	const bool GPS_available, //INPUT: [true or false] flag to indicate when GPS data is available
	const double inclinationAngleDegrees, //INPUT: [degrees] the geomagnetic inclination angle
	const double beta, //INPUT: [<< 1] small-valued tuning parameter 
	const double alpha, //INPUT: [1,2] tuning parameter for magnetometer trust weighting
	const double gamma, //INPUT: [0,1] low-pass filter constant for GPS velocity
	const double fraction //INPUT: [0.5,0.9] fraction of total velocity in the forward direction
);

extern double get_yI(double lambda, double lam0, double phi);
extern double vec4_norm(const double a[4]);
extern double degreesToRadians(double degrees);