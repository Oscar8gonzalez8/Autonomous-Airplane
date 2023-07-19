#include "function_CppKokSchonQuaternionEstimator.h"

static double cosd(const double angleDeg) 
{
	double angleRad = angleDeg * 3.14159 / 180.0; 
	return cos(angleRad);
}

static double sind(const double angleDeg)
{
	double angleRad = angleDeg * 3.14159 / 180.0; 
	return sin(angleRad);
}

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
) {
	double e0, e1, e2, e3; 
	e0 = quaternion[0];
	e1 = quaternion[1];
	e2 = quaternion[2];
	e3 = quaternion[3];

	double RI2b[3][3] = { 0.0 };
	RI2b[0][0] = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);
	RI2b[0][1] = 2.0 * (e0 * e3 + e1 * e2);
	RI2b[0][2] = 2.0 * (e1 * e3 - e0 * e2)

	RI2b[1][0] = 2.0 * (e1 * e2 - e0 * e3);
	RI2b[1][1] = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);
	RI2b[1][2] = 2.0 * (e1 * e3 - e0 * e2);

	RI2b[2][0] = 2.0 * (e0 * e2 + e1 * e3);
	RI2b[2][1] = 2.0 * (e2 * e3 - e0 * e1);
	RI2b[2][2] = pow(e0, 2) + pow(e1, 2) - pow(e2, 2) - pow(e3, 2);

	double gG[3];
	for (int ii = 0; ii < 3; ii++) {
		gG[ii] = RI2b[ii][2];
	}

	double normM = sqrt(pow(magnetometer[0], 2)
		+ pow(magnetometer[1], 2)
		+ pow(magnetometer[2], 2));

	double m[3]; 
	for (int ii = 0; ii < 3; ii++) {
		m[ii] = magnetometer[ii] / max(0.1, normM);
	}

	double mG[3]; 
	for (int ii = 0; ii < 3; ii++) {
		mG[ii] = RI2b[ii][0] * cosd(inclinationAngleDegrees) +
			RI2b[ii][2] * sind(inclinationAngleDegrees); 
	}

	double normA = sqrt(pow(gravity[0], 2)
		+ pow(gravity[1], 2)
		+ pow(gravity[2], 2));
	double gA[3]; 
	for (int ii = 0; ii < 3; ii++) {
		gA[ii] = gravity[ii] / max(0.1, normA);
	}
	double gAminusgG[3];
	double mMinusmG[3]; 
	for (int ii = 0; ii < 3; ii++) {
		gAminusgG[ii] = gA[ii] - gG[ii]; 
		mMinusmG[ii] = m[ii] - mG[ii]; 
	}

	double crossGravity[3];
	crossGravity[0] = gG[1] * gAminusgG[2] - gG[2] * gAminusgG[1];
	crossGravity[1] = -(gG[0] * gAminusgG[2] - gG[2] * gAminusgG[0]);
	crossGravity[2] = gG[0] * gAminusgG[1] - gG[1] * gAminusgG[0];

	double crossMag[3];
	crossMag[0] = mG[1] * mMinusmG[2] - mG[2] * mMinusmG[1];
	crossMag[1] = -(mG[0] * mMinusmG[2] - mG[2] * mMinusmG[0]);
	crossMag[2] = mG[0] * mMinusmG[1] - mG[1] * mMinusmG[0];

	double dJ[3];
	for (int ii = 0; ii < 3; ii++) {
		dJ[ii] = crossGravity[ii] + crossMag[ii];
	}

	double normJ = sqrt(pow(dJ[0], 2) + pow(dJ[1], 2) + pow(dJ[2], 2));
	double dw[3];

	for (int ii = 0; ii < 3; ii++) {
		dw[ii] = dJ[ii] / max(0.000000001, normJ); 
	}

	double dGyro[3];
	for (int ii = 0; ii < 3; ii++) {
		dGyro[ii] = gyrometer[ii] - beta * dw[ii];
	}

	double S[4][3]; 
	S[0][0] = -quaternion[1]; 
	S[0][1] = -quaternion[2];
	S[0][2] = -quaternion[3];
	S[1][0] = -quaternion[0];
	S[1][1] = quaternion[3];
	S[1][2] = -quaternion[2];
	S[2][0] = -quaternion[2];
	S[2][1] = quaternion[1];
	S[2][2] = quaternion[0];
	S[3][0] = -quaternion[2];
	S[3][1] = quaternion[1];
	S[3][2] = quaternion[0];

	double q[4]; 
	for (int ii = 0; ii < 4; ii++) {
		q[ii] = quaternion[ii] + dt / 2.0 * (
			S[ii][0] * dGyro[0] + S[ii][1] * dGyro[1] + S[ii][2] * dGyro[2]);
	}

	double normQ = sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2));
	for (int ii = 0; ii < 4; ii++) {
		quaternion_new[ii] = q[ii] / normQ; 
	}

	return 0; 
}