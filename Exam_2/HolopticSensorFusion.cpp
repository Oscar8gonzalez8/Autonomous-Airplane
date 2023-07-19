#include "HolopticSensorFusion.h"
#include <math.h>
#include <cmath>

using namespace std;

// radius of the earth 
const double rEarth = 6371000; 

//Run the Holoptic Sensor Fusion Algorithm
extern int runHolopticSensorFusion(
	double quaternion_new[4], //OUTPUT: 'q' in matlab, updated quaternion orientation
	double velocities[3], //OUTPUT: XI_new [m/s] updated body-frame velocities estimate
	double GPS_velocities[3], //OUTPUT: VI_new [m/s] updated inertial-frame velocities estimate
	double GPS_positions[3], //OUTPUT: lat_new,long_new, alt_new [m] updated inertial positions estimate
	// double* t_GPS_new, //OUTPUT: [s] the most recent timestamp for new GPS data
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
	const double inclinationAngleDegrees, //INPUT: [degrees] the geomagnetic inclination angle
	const double dt, //INPUT: [s] time-step
	const double dtGPS, //INPUT: [s] time-step of GPS/altitude measurements
	const double beta, //INPUT: [<< 1] small-valued tuning parameter 
	const double alpha, //INPUT: [1,2] tuning parameter for magnetometer trust weighting
	const double gamma, //INPUT: [0,1] low-pass filter constant for GPS velocity
	const double fraction, //INPUT: [0.5,0.9] fraction of total velocity in the forward direction
	const bool GPS_available //INPUT: [true or false] flag to indicate when GPS data is available
) {
	static double aLP[3] = { 0, 0, 0 };  // Initialized to [0, 0, 0]

	// Convert GPS and waypoint data to radians
	double phi;
	phi = degreesToRadians(GPS_latitude);
	double lambda;
	lambda = degreesToRadians(GPS_longitude);
	double phi0;
	phi0 = degreesToRadians(GPS_latitude_origin);
	double lam0;
	lam0 = degreesToRadians(GPS_longitude_origin);

	// Get inertial displacements 
	double xI = rEarth * (phi - phi0);
	double yI = get_yI(lambda, lam0, phi);
	double zI = -(altitude - altitude_origin);
	double XI[3] = { xI, yI, zI };
	double VI_new[3];

	// North-East distance traveled
	// xyDist = sqrt(xI^2+yI^2);
	double xyDist = sqrt(xI*xI + yI*yI);

	double dtGPS_safe;
	dtGPS_safe = max(dtGPS, 0.001);
	if (xyDist > 0.1) {
		// New GPS is available
		//double dtGPS; 
		//dtGPS = t_now - t_GPS; // (s) GPS timestep
		//*t_GPS_new = t_now; // (s) update the GPS time

		for (int i = 0; i < 3; ++i) {
			// prevent divide by zero
			
			VI_new[i] = (XI[i] - GPS_positions_old[i]) / dtGPS_safe; // (m/s) inertial velocity
		}
	}
	else {

		for (int i = 0; i < 3; ++i) {
			VI_new[i] = GPS_velocities_old[i]; // (m/s) inertial velocity
		}
	}

	// Update XI_new
	double XI_new[3];
	for (int i = 0; i < 3; ++i) {
		XI_new[i] = XI[i]; // (m/s) inertial velocity
	}
		
	
	// get the rotation matrix from the inertial to body frames 
	double RI2b[3][3]; 
	double e0 = quaternion[0];
	double e1 = quaternion[1];
	double e2 = quaternion[2];
	double e3 = quaternion[3];
	Rotation_I2b(RI2b, e0, e1, e2, e3);

	// get the body - frame velocities
	double normVI_new; 
	normVI_new = sqrt(GPS_velocities[0] * GPS_velocities[0] +
		GPS_velocities[1] * GPS_velocities[1] +
		GPS_velocities[2] * GPS_velocities[2]);
	// similar matlab code
	// Vbody = [norm(VI_new); 0; 0];
	double Vbody[3];
	Vbody[0] = normVI_new;
	Vbody[1] = 0.0;
	Vbody[2] = 0.0;

	// low - pass filter the accelerometer signal
	double tau = 0.02; // (s)low - pass filter time constant
	for (int i = 0; i < 3; i++) {
		aLP[i] = (1 - dt / max(dt, tau)) * aLP[i] + dt / max(dt, tau) * accelerometer[i]; // (m / s2)
	}
	

	// accelerometer's estimate of gravity
	 double g_ak[3]; 
	 double norm_g_ak[3];
	 double crossProduct[3];
	 vec3_cross(crossProduct, gyrometer, Vbody);
	 // Now you can use crossProduct in other calculations
	 //for (int i = 0; i < 3; i++) {
		// g_ak[i] = aLP[i] + crossProduct[i];
	 //}
	 vec3_norm(g_ak, accelerometer);
	 //vec4_norm(norm_g_ak, g_ak);

	// get the normalized gravity vector 
	//double norm_g[3]; 
	//vec3_norm(norm_g, accelerometer);

	double gGyro[3];
	double mGyro[3];
	double vec1[3] = { 0, 0, 1 };
	Matrix3_Vec3_Multiplication(gGyro, RI2b, vec1);

	double inc_angle_rad = degreesToRadians(inclinationAngleDegrees);
	double vec[3] = { cos(inc_angle_rad), 0, sin(inc_angle_rad) };
	Matrix3_Vec3_Multiplication(mGyro, RI2b, vec);

	// Magnetometer's measurement of geomagnetic vector
	double m[3]; 
	vec3_norm(m, magnetometer);

	// Calculate the Kok Schon gradient
	double gradient[3]; 

	KokSchonGradient(gradient,
		g_ak, //aka g_ak accelerometer's estimate of gravity
		m, // aka 'm' Magnetometer's measurement of geomagnetic vector 
		mGyro, // Gyrometer's estimate of the geomagnetic vector
		gGyro, // Gyrometer's estimate of gravity
		alpha);


	KokSchonUpdate(
		quaternion_new,
		quaternion,
		gradient,
		gyrometer,
		dt,
		beta);

	//Estimate the body-frame velocities
	velocities[0] = Vbody[0];
	velocities[1] = Vbody[1];
	velocities[2] = Vbody[2];

	//Estimate the inertial-frame velocities
	GPS_velocities[0] = VI_new[0];
	GPS_velocities[1] = VI_new[1];
	GPS_velocities[2] = VI_new[2];

	//Estimate the inertial-frame displacements
	GPS_positions[0] = XI_new[0];
	GPS_positions[1] = XI_new[1];
	GPS_positions[2] = XI_new[2];


	//If any output is infinite or NaN, reset everything
	if (isinf(quaternion_new[0]) || isnan(quaternion_new[0])
		|| isinf(quaternion_new[1]) || isnan(quaternion_new[1])
		|| isinf(quaternion_new[2]) || isnan(quaternion_new[2])
		|| isinf(quaternion_new[3]) || isnan(quaternion_new[3])
		|| isinf(velocities[0]) || isnan(velocities[0])
		|| isinf(velocities[1]) || isnan(velocities[1])
		|| isinf(velocities[2]) || isnan(velocities[2])
		|| isinf(GPS_velocities[0]) || isnan(GPS_velocities[0])
		|| isinf(GPS_velocities[1]) || isnan(GPS_velocities[1])
		|| isinf(GPS_velocities[2]) || isnan(GPS_velocities[2])
		|| isinf(GPS_positions[0]) || isnan(GPS_positions[0])
		|| isinf(GPS_positions[1]) || isnan(GPS_positions[1])
		|| isinf(GPS_positions[2]) || isnan(GPS_positions[2])
		//|| isinf(*t_GPS_new) || isnan(*t_GPS_new)
		)
	{
		//Some proplem happened, reset everything
		//Reset quaternions
		quaternion_new[0] = 1.0f;
		for (unsigned int ii = 1; ii < 4; ii++)
			quaternion_new[ii] = 0.0f;

		//Reset everything else to zero
		for (unsigned int ii = 0; ii < 3; ii++) {
			velocities[ii] = 0.0f;
			GPS_velocities[ii] = 0.0f;
			GPS_positions[ii] = 0.0f;
		}

		return 1;  //indicate that something failed
	}
	else {
		return 0; //normal case
	}
}

//Create a cross-product matrix
extern void CrossMatrix(
	double CrossMat[3][3],//OUTPUT: Cross product matrix
	const double vec3[3])//INPUT: vector to be converted to a cross product matrix
{
	CrossMat[0][0] = 0;
	CrossMat[0][1] = -vec3[2];
	CrossMat[0][2] = vec3[1];
	CrossMat[1][0] = vec3[2];
	CrossMat[1][1] = 0;
	CrossMat[1][2] = -vec3[0];
	CrossMat[2][0] = -vec3[1];
	CrossMat[2][1] = vec3[0];
	CrossMat[2][2] = 0;
}

extern void Rotation_I2b(
	double RI2b[3][3],
	const double e0,
	const double e1,
	const double e2,
	const double e3)
{
	RI2b[0][0] = e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3;
	RI2b[0][1] = 2 * (e0 * e3 + e1 * e2);
	RI2b[0][2] = 2 * (e1 * e3 - e0 * e2);

	RI2b[1][0] = 2 * (e1 * e2 - e0 * e3);
	RI2b[1][1] = e0 * e0 - e1 * e1 + e2 * e2 - e3 * e3;
	RI2b[1][2] = 2 * (e0 * e1 + e2 * e3);

	RI2b[2][0] = 2 * (e0 * e2 + e1 * e3);
	RI2b[2][1] = 2 * (e2 * e3 - e0 * e1);
	RI2b[2][2] = e0 * e0 - e1 * e1 - e2 * e2 + e3 * e3;
}

//void inertial_2_body_Rotation(
//	double* x_b,
//	double* y_b,
//	double* z_b,
//	const double xI,
//	const double yI,
//	const double zI,
//	const double e0,
//	const double e1,
//	const double e2,
//	const double e3)
//{
//	// insert code here
//}

extern void vec4_norm(
	double C[4],
	const double a[4])
{
	double norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
	// prevent divide by zero
	norm = max(norm, 0.001);
	C[0] = a[0] / norm;
	C[1] = a[1] / norm;
	C[2] = a[2] / norm;
	C[3] = a[3] / norm; 
}

extern void vec3_norm(
	double C[3],
	const double a[3])
{
	double norm = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
	// prevent divide by zero
	norm = max(norm, 0.001);
	C[0] = a[0] / norm;
	C[1] = a[1] / norm;
	C[2] = a[2] / norm;
}

extern void Matrix3_Vec3_Multiplication(
	double c[3],
	const double A[3][3],
	const double b[3])
{
	for (int i = 0; i < 3; ++i) {
		c[i] = 0.0;
		for (int j = 0; j < 3; ++j) {
			c[i] += A[i][j] * b[j];
		}
	}
}

extern void Matrix4_Vec3_Multiplication(
	double c[4],
	const double A[4][3],
	const double b[3])
{
	for (int i = 0; i < 4; ++i) {
		c[i] = 0.0;
		for (int j = 0; j < 3; ++j) {
			c[i] += A[i][j] * b[j];
		}
	}
}

extern void Vec3_Scalar_Multiplication(
	double c[3],
	const double b[3],
	const double a)
{
	c[0] = a * b[0];
	c[1] = a * b[1];
	c[2] = a * b[2];
}

extern void vec3_cross(
	double C[3],
	const double a[3],
	const double b[3])
{
	C[0] = a[1] * b[2] - a[2] * b[1];
	C[1] = a[2] * b[0] - a[0] * b[2];
	C[2] = a[0] * b[1] - a[1] * b[0];
}

extern void vec3_addition(
	double c[3],
	const double a[3],
	const double b[3])
{
	c[0] = a[0] + b[0];
	c[1] = a[1] + b[1];
	c[2] = a[2] + b[2];
}

//subtract one 3d vector from another c=a-b
extern void vec3_subtraction(
	double c[3],
	const double a[3],
	const double b[3])
{
	c[0] = a[0] - b[0];
	c[1] = a[1] - b[1];
	c[2] = a[2] - b[2];
}

extern void KokSchonGradient(
	double gradient[3],
	const double NormalizedAccelerometer[3], //aka g_ak accelerometer's estimate of gravity
	const double NormalizedMagnetometer[3], // aka 'm' Magnetometer's measurement of geomagnetic vector 
	const double mGyro[3], // Gyrometer's estimate of the geomagnetic vector
	const double gGyro[3], // Gyrometer's estimate of gravity
	const double alpha)
{
	// Calculate the Kok Schon gradient
	// gradient = cross(gGyro, (NormalizedAccelerometer - gGyro)) + alpha * cross(mGyro, (NormalizedMagnetometer - mGyro));
	double accel_minus_gGyro[3], mag_minus_mGyro[3], cross1[3], cross2[3];

	vec3_subtraction(accel_minus_gGyro, NormalizedAccelerometer, gGyro);
	vec3_subtraction(mag_minus_mGyro, NormalizedMagnetometer, mGyro);

	// calculate the cross products
	vec3_cross(cross1, gGyro, accel_minus_gGyro);

	vec3_cross(cross2, mGyro, mag_minus_mGyro);

	// multiply the second cross product by alpha
	Vec3_Scalar_Multiplication(cross2, cross2, alpha);

	// sum the two cross products to get the gradient
	vec3_addition(gradient, cross1, cross2);
}

extern void KokSchonUpdate(
	double quaternion_new[4],
	const double quaternion[4],
	const double gradient[3],
	const double gyrometer[3],
	const double dt,
	const double beta)
{

	// Get the angular velocity update
	double norm_gradient[3];
	vec3_norm(norm_gradient, gradient);
	double dw[3];
	for (int i = 0; i < 3; ++i) {
		dw[i] = beta * norm_gradient[i];
	}

	double e0 = quaternion[0];
	double e1 = quaternion[1];
	double e2 = quaternion[2];
	double e3 = quaternion[3];

	double Sk[4][3] = {
	{-e1, -e2, -e3},
	{e0, -e3, e2},
	{e3, e0, -e1},
	{-e2, e1, e0}
	};

	// gyro - dw
	double gyrometer_minus_dw[3];
	for (int i = 0; i < 3; ++i) {
		gyrometer_minus_dw[i] = gyrometer[i] - dw[i];
	}

	// Sk*(gyro - dw)
	double Sk_times_gyro_minus_dw[4];
	Matrix4_Vec3_Multiplication(Sk_times_gyro_minus_dw, Sk, gyrometer_minus_dw);

	// qNew = q_old + dt/2*Sk*(gyro - dw)
	for (int i = 0; i < 4; ++i) {
		quaternion_new[i] = quaternion[i] + (dt / 2.0) * Sk_times_gyro_minus_dw[i];
	}

	// q = qNew / norm(qNew)
	double norm_quat_new = vec4_norm(quaternion_new);
	// prevent divide by zero
	norm_quat_new = max(norm_quat_new, 0.001);
	for (int i = 0; i < 4; ++i) {
		quaternion_new[i] /= norm_quat_new;
	}
}

extern double degreesToRadians(double degrees) {
	return degrees * (PI / 180.0);
}

extern double vec4_norm(const double a[4])
{
	return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
}

extern double get_yI(double lambda, double lam0, double phi) {
	double dlambda;
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

	double yI = (dlambda > 0 ? 1 : -1) * rEarth * acos((cos(dlambda) - 1) * pow(cos(phi), 2) + 1);
	return yI;
}