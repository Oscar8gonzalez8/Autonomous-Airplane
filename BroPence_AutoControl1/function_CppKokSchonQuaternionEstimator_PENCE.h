#pragma once

#include <math.h>
#include <cmath>

#define max(a,b) ((a<b)?b:a)
#define min(a,b) ((a>b)?b:a)

extern void CrossMatrix(
	double CrossMat[3][3],
	const double vec3[3]
);

extern void Rotation_I2b(
	double Ri2b[3][3],
	const double e0,
	const double e1,
	const double e2,
	const double e3
	);

double function_KokSchonQuaternionEstimator(double quaternion_new[4], const double quaternion[4],
	const double gyrometer[3], const double gravity[3],
	const double magnetometer[3], const double dt,
	const double inclinationAngleDegrees, const double alpha,
	const double beta);

void CrossMatrix(double CrossMat[3][3], const double vec[3]);
void Rotation_I2b(double Ri2b[3][3], const double e0, const double e1, const double e2, const double e3);

void inertial_2_body_Rotation(double* x_b, double* y_b, double* z_b, const double xI, const double yI, const double zI,
	const double e0, const double e1, const double e2, const double e3);

void body_2_Inertial_Rotation(double* xI, double* yI, double* zI, const double xb, const double yb, const double zb,
	const double e0, const double e1, const double e2, const double e3);

void vec4_norm(double C[4], const double a[4]);

void vec3_norm(double C[3], const double a[3]);

double norm3(double x[3]);

void Matrix_3_Vec3_Multplication(double c[3], const double A[3][3], const double b[3]);

void Matrix_3_Scaler_Multplication(double c[3], const double b[3], const double a);

void vec3_cross(double C[3], const double a[3], const double b[3]);

void vec3_addition(double c[3], const double a[3], const double b[3]);

void vec3_subtraction(double c[3], const double a[3], const double b[3]);

void QuaternionTimeDerivative(double dq[4], const double q[4], const double omega[3]);

void KokSchonUpdate(
	double quaternion_new[4],
	const double quaternion[4],
	const double gradient[3],
	const double gyrometer[3],
	const double dt,
	const double beta);

void KokSchonGradient(double gradient[3], const double NormalizedAccelerometer[3], const double NormalizedMagnetometer[3],
	const double quaternion[4], const double inclinationAngleDegrees, const double alpha);

//extern void inertial_2_body_Rotation(
//	double* x_b,
//	double* y_b,
//	double* z_b,
//	const double xI,
//	const double yI,
//	const double zI,
//	const double e0,
//	const double e1,
//	const double e2,
//	const double e3,
//	);


//static double cosd(const double angleDeg);
//static double sind(const double angleDeg);
//// The Kok Schon Quaternion Estimator Algorithm 
//extern int function_KokSchonQuaternionEstimator(
//	double quaternion_new[4],
//	const double quaternion[4],
//	const double gyrometer[3],
//	const double gravity[3],
//	const double magnetometer[3],
//	const double dt,
//	const double inclinationAngleDegrees,
//	const double alpha,
//	const double beta
//);