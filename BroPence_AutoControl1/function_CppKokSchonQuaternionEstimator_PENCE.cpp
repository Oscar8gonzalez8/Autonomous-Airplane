#include "function_CppKokSchonQuaternionEstimator_PENCE.h"
#include <cmath>

void CrossMatrix(double CrossMat[3][3], const double vec[3]) {
    CrossMat[0][0] = 0.0;
    CrossMat[0][1] = -vec[2];
    CrossMat[0][2] = vec[1];
    CrossMat[1][0] = vec[2];
    CrossMat[1][1] = 0.0;
    CrossMat[1][2] = -vec[0];
    CrossMat[2][0] = -vec[1];
    CrossMat[2][1] = vec[0];
    CrossMat[2][2] = 0.0;
}

void Rotation_I2b(double Ri2b[3][3], const double e0, const double e1, const double e2, const double e3) {
    Ri2b[0][0] = e0 * e0 + e1 * e1 - e2 * e2 - e3 * e3;
    Ri2b[0][1] = 2.0 * (e0 * e3 - e1 * e2);
    Ri2b[0][2] = 2.0 * (e1 * e3 + e0 * e2);
    Ri2b[1][0] = 2.0 * (e1 * e2 + e0 * e3);
    Ri2b[1][1] = e0 * e0 - e1 * e1 + e2 * e2 - e3 * e3;
    Ri2b[1][2] = 2.0 * (e2 * e3 - e0 * e1);
    Ri2b[2][0] = 2.0 * (e1 * e3 - e0 * e2);
    Ri2b[2][1] = 2.0 * (e0 * e1 + e2 * e3);
    Ri2b[2][2] = e0 * e0 - e1 * e1 - e2 * e2 + e3 * e3;
}

void inertial_2_body_Rotation(double* x_b, double* y_b, double* z_b, const double xI, const double yI, const double zI,
    const double e0, const double e1, const double e2, const double e3) {
    double e0_2 = e0 * e0;
    double e1_2 = e1 * e1;
    double e2_2 = e2 * e2;
    double e3_2 = e3 * e3;
    double e0e1 = e0 * e1;
    double e0e2 = e0 * e2;
    double e0e3 = e0 * e3;
    double e1e2 = e1 * e2;
    double e1e3 = e1 * e3;
    double e2e3 = e2 * e3;

    *x_b = (e0_2 + e1_2 - e2_2 - e3_2) * xI + 2.0 * (e0e3 + e1e2) * yI + 2.0 * (e1e3 - e0e2) * zI;
    *y_b = 2.0 * (e1e2 - e0e3) * xI + (e0_2 - e1_2 + e2_2 - e3_2) * yI + 2.0 * (e0e1 + e2e3) * zI;
    *z_b = 2.0 * (e0e2 + e1e3) * xI + 2.0 * (e2e3 - e0e1) * yI + (e0_2 - e1_2 - e2_2 + e3_2) * zI;
}

void body_2_Inertial_Rotation(double* xI, double* yI, double* zI, const double xb, const double yb, const double zb,
    const double e0, const double e1, const double e2, const double e3) {
    double e0_2 = e0 * e0;
    double e1_2 = e1 * e1;
    double e2_2 = e2 * e2;
    double e3_2 = e3 * e3;
    double e0e1 = e0 * e1;
    double e0e2 = e0 * e2;
    double e0e3 = e0 * e3;
    double e1e2 = e1 * e2;
    double e1e3 = e1 * e3;
    double e2e3 = e2 * e3;

    *xI = (e0_2 + e1_2 - e2_2 - e3_2) * xb + 2.0 * (-e0e3 + e1e2) * yb + 2.0 * (e1e3 + e0e2) * zb;
    *yI = 2.0 * (e1e2 + e0e3) * xb + (e0_2 - e1_2 + e2_2 - e3_2) * yb + 2.0 * (-e0e1 + e2e3) * zb;
    *zI = 2.0 * (-e0e2 + e1e3) * xb + 2.0 * (e2e3 + e0e1) * yb + (e0_2 - e1_2 - e2_2 + e3_2) * zb;
}

void vec4_norm(double C[4], const double a[4]) {
    double den = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2] + a[3] * a[3]);
    if (den > 0.000001) {
        C[0] = a[0] / den;
        C[1] = a[1] / den;
        C[2] = a[2] / den;
        C[3] = a[3] / den;
    }
    else {
        C[0] = a[0];
        C[1] = a[1];
        C[2] = a[2];
        C[3] = a[3];
    }
}

void vec3_norm(double C[3], const double a[3]) {
    double den = sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (den > 0.000001) {
        C[0] = a[0] / den;
        C[1] = a[1] / den;
        C[2] = a[2] / den;
    }
    else {
        C[0] = a[0];
        C[1] = a[1];
        C[2] = a[2];
    }
}

double norm3(double x[3]) {
    return sqrt(x[0] * x[0] + x[1] * x[1] + x[2] * x[2]);
}

void Matrix_3_Vec3_Multplication(double c[3], const double A[3][3], const double b[3]) {
    for (int ii = 0; ii < 3; ii++) {
        c[ii] = 0.0;
        for (int jj = 0; jj < 3; jj++) {
            c[ii] += A[ii][jj] * b[jj];
        }
    }
}

void Matrix_3_Scaler_Multplication(double c[3], const double b[3], const double a) {
    for (int ii = 0; ii < 3; ii++) {
        c[ii] = b[ii] * a;
    }
}

void vec3_cross(double C[3], const double a[3], const double b[3]) {
    C[0] = a[1] * b[2] - a[2] * b[1];
    C[1] = a[2] * b[0] - a[0] * b[2];
    C[2] = a[0] * b[1] - a[1] * b[0];
}

void vec3_addition(double c[3], const double a[3], const double b[3]) {
    c[0] = a[0] + b[0];
    c[1] = a[1] + b[1];
    c[2] = a[2] + b[2];
}

void vec3_subtraction(double c[3], const double a[3], const double b[3]) {
    c[0] = a[0] - b[0];
    c[1] = a[1] - b[1];
    c[2] = a[2] - b[2];
}

void QuaternionTimeDerivative(double dq[4], const double q[4], const double omega[3]) {
    dq[0] = -q[1] * omega[0] - q[2] * omega[1] - q[3] * omega[2];
    dq[1] = q[0] * omega[0] - q[3] * omega[1] + q[2] * omega[2];
    dq[2] = q[3] * omega[0] + q[0] * omega[1] - q[1] * omega[2];
    dq[3] = -q[2] * omega[0] + q[1] * omega[1] + q[0] * omega[2];
}

void KokSchonUpdate(
    double quaternion_new[4],
    const double quaternion[4],
    const double gradient[3],
    const double gyrometer[3],
    const double dt,
    const double beta) {
    double q0 = quaternion[0];
    double q1 = quaternion[1];
    double q2 = quaternion[2];
    double q3 = quaternion[3];

    double normGrad = sqrt(gradient[0] * gradient[0] + gradient[1] * gradient[1] + gradient[2] * gradient[2]);

    double RI2b[3][3];
    Rotation_I2b(RI2b, q0, q1, q2, q3);

    double unitGravity[3];
    vec3_norm(unitGravity, gyrometer);

    double unitMag[3];
    vec3_norm(unitMag, gyrometer);

    double omega[3];
    if (normGrad > 0.000001) {
        for (int ii = 0; ii < 3; ii++) {
            omega[ii] = gyrometer[ii] - beta * gradient[ii];
        }
    }
    else {
        for (int ii = 0; ii < 3; ii++) {
            omega[ii] = gyrometer[ii];
        }
    }

    double qu[4];
    qu[0] = q0 + 0.5 * dt * (-q1 * omega[0] - q2 * omega[1] - q3 * omega[2]);
    qu[1] = q1 + 0.5 * dt * (q0 * omega[0] - q3 * omega[1] + q2 * omega[2]);
    qu[2] = q2 + 0.5 * dt * (q3 * omega[0] + q0 * omega[1] - q1 * omega[2]);
    qu[3] = q3 + 0.5 * dt * (-q2 * omega[0] + q1 * omega[1] + q0 * omega[2]);

    double norm_qu = sqrt(qu[0] * qu[0] + qu[1] * qu[1] + qu[2] * qu[2] + qu[3] * qu[3]);

    if (norm_qu > 0.001) {
        for (int ii = 0; ii < 4; ii++) {
            quaternion_new[ii] = qu[ii] / norm_qu;
        }
    }
    else {
        quaternion_new[0] = 1.0;
        quaternion_new[1] = 0.0;
        quaternion_new[2] = 0.0;
        quaternion_new[3] = 0.0;
    }
}

void KokSchonGradient(double gradient[3], const double NormalizedAccelerometer[3], const double NormalizedMagnetometer[3],
    const double quaternion[4], const double inclinationAngleDegrees, const double alpha) {
    double RI2b[3][3];
    double gravityInBodyFrame[3];
    double MagInBodyFrame[3];
    double inclinationAngleRadians = inclinationAngleDegrees * 3.14159 / 180.0;
    double cos_incAngle = cos(inclinationAngleRadians);
    double sin_incAngle = sin(inclinationAngleRadians);
    double accelMinusGravity[3];
    double MagMinusMag[3];
    double GravityCrossGravityError[3];
    double MagCrossMagError[3];

    Rotation_I2b(RI2b, quaternion[0], quaternion[1], quaternion[2], quaternion[3]);

    for (int ii = 0; ii < 3; ii++) {
        gravityInBodyFrame[ii] = RI2b[ii][2];
    }

    for (int ii = 0; ii < 3; ii++) {
        MagInBodyFrame[ii] = RI2b[ii][0] * cos_incAngle + RI2b[ii][2] * sin_incAngle;
    }

    for (int ii = 0; ii < 3; ii++) {
        accelMinusGravity[ii] = NormalizedAccelerometer[ii] - gravityInBodyFrame[ii];
    }

    for (int ii = 0; ii < 3; ii++) {
        MagMinusMag[ii] = NormalizedMagnetometer[ii] - MagInBodyFrame[ii];
    }

    vec3_cross(GravityCrossGravityError, gravityInBodyFrame, accelMinusGravity);
    vec3_cross(MagCrossMagError, MagInBodyFrame, MagMinusMag);

    for (int ii = 0; ii < 3; ii++) {
        gradient[ii] = GravityCrossGravityError[ii] + alpha * MagCrossMagError[ii];
    }
}

double function_KokSchonQuaternionEstimator(double quaternion_new[4], const double quaternion[4],
    const double gyrometer[3], const double gravity[3],
    const double magnetometer[3], const double dt,
    const double inclinationAngleDegrees, const double alpha,
    const double beta) {
    double e0 = quaternion[0];
    double e1 = quaternion[1];
    double e2 = quaternion[2];
    double e3 = quaternion[3];

    double norm_q = sqrt(e0 * e0 + e1 * e1 + e2 * e2 + e3 * e3);
    if (fabs(norm_q) > 0.0001) {

        e0 = e0 / norm_q;
        e1 = e1 / norm_q;
        e2 = e2 / norm_q;
        e3 = e3 / norm_q;
    }
    else {
        e0 = 1.0;
        e1 = 0.0;
        e2 = 0.0;
        e3 = 0.0;
    }

    double RI2b[3][3];
    Rotation_I2b(RI2b, e0, e1, e2, e3);

    double unitGravity[3];
    vec3_norm(unitGravity, gravity);

    double unitMag[3];
    vec3_norm(unitMag, magnetometer);

    double gradient[3];

    KokSchonGradient(gradient, unitGravity, unitMag, quaternion, inclinationAngleDegrees, alpha);

    KokSchonUpdate(quaternion_new, quaternion, gradient, gyrometer, dt, beta);

    if (isinf(quaternion_new[0]) || isnan(quaternion_new[0]) ||
        isinf(quaternion_new[1]) || isnan(quaternion_new[1]) ||
        isinf(quaternion_new[2]) || isnan(quaternion_new[2]) ||
        isinf(quaternion_new[3]) || isnan(quaternion_new[3])) {
        quaternion_new[0] = 1.0;
        quaternion_new[1] = 0.0;
        quaternion_new[2] = 0.0;
        quaternion_new[3] = 0.0;
        return 1;
    }
    else {
        return 0;
    }
}

