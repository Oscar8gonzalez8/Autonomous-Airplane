//Include the Matlab Executable (mex) header files to use the following:
//mex, MexFunction, matlab::mex::Function, matlab::data,
//matlab::mex::ArgumentList, getNumberOfElements(), MATLABEngine(),
//TypedArray, ArrayFactory
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "HolopticSensorFusion_v1.h"


//To use TypedArray, ArrayFactory, ArrayType, Array, ArrayDimensions,
//without matlab::data::
using namespace matlab::data; 
using matlab::mex::ArgumentList; //To use ArgumentList without matlab::mex::


void convertMatlabInput(double* input, TypedArray<double> M_array) {
	int ii = 0;
	for (auto& elem : M_array) {
		input[ii] = elem; //Populate the C++ input
		ii++;
	}
}

//The class MexFunction which inherits from matlab::mex::Function
// is required for mex functions that use C++
class MexFunction : public matlab::mex::Function {
public:
	//Declare the inputs from Matlab and the Outputs from C++ as
	// ArgumentList variables
	void operator()(ArgumentList outputs, ArgumentList inputs) {

		//input variables
		double q[4] = { 0.0 }; //Quaternion orientation
		double VI[3]; //[VxI;VyI;VzI] (m/s) previous estimated inertial frame velocities
		double XI[3]; //[xI; yI; zI](m) previous estimated inertial frame positions
		double gyro[3]; //[wx; wy; wz](rad / s) angular velocities
		double accel[3]; //[ax; ay; az](m / s2) gravity positive, linear accelerations negative
		double magnet[3];//[Mx; My; Mz](uT) geomagnetic field strengths
		double GPS_lat; //[lat](deg) latitude angle
		double GPS_lon; //[lon](deg) longitude angle
		double GPS_alt; //[-zI](m) altitude
		double lat0; //[lat](deg) initial latitude angle
		double lon0; //[lon](deg) initial longitude angle
		double alt0; //[-zI](m) initial altitude
		double t_now; //INPUT: [s] present run-time (time right now)
		double t_GPS; //[t](s) time (present runtime)
		double dt; //[dt](s) iteration time step
		bool GPS_available; //[0 or 1] flag indicating whether new GPS is available or not
		double inclinationAngleDegrees; //INPUT: [degrees] the geomagnetic inclination angle
		double beta; //INPUT: [<< 1] small-valued tuning parameter 
		double alpha; //INPUT: [1,2] tuning parameter for magnetometer trust weighting
		double gamma; //INPUT: [0,1] low-pass filter constant for GPS velocity
		double fraction; //INPUT: [0.5,0.9] fraction of total velocity in the forward direction

		//Output variables
		double qNew[4] = { 0.0 }; //[e0; e1; e2; e3](-1 to 1) estimated quaternions
		qNew[0] = 1.0;
		double VInew[3] = { 0.0 }; //[VxI; VyI; VzI](m / s) estimated inertial frame velocities
		double XInew[3] = { 0.0 }; //[xI; yI; zI](m) estimated inertial frame positions
		double t_GPS_new = 0.0; //OUTPUT: [s] the most recent timestamp for new GPS data

		//Get the MATLAB input variables
		TypedArray<double> m_q = std::move(inputs[0]);
		TypedArray<double> m_VI = std::move(inputs[1]);
		TypedArray<double> m_XI = std::move(inputs[2]);
		TypedArray<double> m_gyro = std::move(inputs[3]);
		TypedArray<double> m_accel = std::move(inputs[4]);
		TypedArray<double> m_magnet = std::move(inputs[5]);
		TypedArray<double> m_GPS_lat = std::move(inputs[6]);
		TypedArray<double> m_GPS_lon = std::move(inputs[7]);
		TypedArray<double> m_GPS_alt = std::move(inputs[8]);
		TypedArray<double> m_lat0 = std::move(inputs[9]);
		TypedArray<double> m_lon0 = std::move(inputs[10]);
		TypedArray<double> m_alt0 = std::move(inputs[11]);
		TypedArray<double> m_t_now = std::move(inputs[12]);
		TypedArray<double> m_t_GPS = std::move(inputs[13]);
		TypedArray<double> m_dt = std::move(inputs[14]);
		TypedArray<double> m_newGPS = std::move(inputs[15]);
		TypedArray<double> m_inclinationAngleDegrees = std::move(inputs[16]);
		TypedArray<double> m_beta = std::move(inputs[17]);
		TypedArray<double> m_alpha = std::move(inputs[18]);
		TypedArray<double> m_gamma = std::move(inputs[19]);
		TypedArray<double> m_fraction = std::move(inputs[20]);


		//Convert the Matlab input variables to C++ variables
		convertMatlabInput(q, m_q);
		convertMatlabInput(VI, m_VI);
		convertMatlabInput(XI, m_XI);
		convertMatlabInput(gyro, m_gyro);
		convertMatlabInput(accel, m_accel);
		convertMatlabInput(magnet, m_magnet);
		GPS_lat = m_GPS_lat[0];
		GPS_lon = m_GPS_lon[0];
		GPS_alt = m_GPS_alt[0];
		lat0 = m_lat0[0];
		lon0 = m_lon0[0];
		alt0 = m_alt0[0];
		t_now = m_t_now[0];
		t_GPS = m_t_GPS[0];
		dt = m_dt[0];
		GPS_available = (bool)m_newGPS[0];
		inclinationAngleDegrees = m_inclinationAngleDegrees[0];
		beta = m_beta[0]; 
		alpha = m_alpha[0]; 
		gamma = m_gamma[0];
		fraction = m_fraction[0];

		double Vbody[3];
		//Run the Holoptic sensor fusion algorithm
		runHolopticSensorFusion(
			qNew, //OUTPUT: updated quaternion orientation
			Vbody, //OUTPUT: [m/s] updated body-frame velocities estimate
			VInew, //OUTPUT: [m/s] updated inertial-frame velocities estimate
			XInew, //OUTPUT: [m] updated inertial positions estimate
			&t_GPS_new, //OUTPUT: [s] the most recent timestamp for new GPS data
			q, //INPUT: previous quaternion orientation
			VI, //INPUT: [m/s] previous estimate of inertial-frame velocities
			XI, //INPUT: [m] previous estimate of inertial-frame positions
			gyro, //INPUT: [rad/s] 3-axis gyro data with bias removed
			accel, //INPUT: [m/s2] x-front, y-right, z-down raw accelerometer signal, bias removed
			magnet, //INPUT: [any] x-front, y-right, z-down calibrated magnetometer signal any units, bias removed
			GPS_lat, //INPUT: [degrees] GPS latitude
			GPS_lon, //INPUT: [degrees] GPS longitude
			GPS_alt, //INPUT: [m] GPS altitude or barometric pressure altitude
			lat0, //INPUT: [degrees] latitude at the origin [0,0,0]
			lon0, //INPUT: [degrees] longitude at the origin [0,0,0]
			alt0, //INPUT: [m] altitude at the origin [0,0,0]
			t_now, //INPUT: [s] present run-time (time right now)
			t_GPS, //INPUT: [s] last time new GPS data arrived
			dt, //INPUT: [s] time-step
			GPS_available, //INPUT: [true or false] flag to indicate when GPS data is available
			inclinationAngleDegrees, //INPUT: [degrees] the geomagnetic inclination angle
			beta, //INPUT: [<< 1] small-valued tuning parameter 
			alpha, //INPUT: [1,2] tuning parameter for magnetometer trust weighting
			gamma, //INPUT: [0,1] low-pass filter constant for GPS velocity
			fraction //INPUT: [0.5,0.9] fraction of total velocity in the forward direction
		);

		
		//Create the output Matlab array
		ArrayFactory factory;

		//Create the Matlab output variables
		TypedArray<double> m_qNew =
			factory.createArray<double>({ 4,1 }, { qNew[0], qNew[1], qNew[2], qNew[3] });
		TypedArray<double> m_VInew =
			factory.createArray<double>({ 3,1 }, { VInew[0], VInew[1], VInew[2] });
		TypedArray<double> m_XInew =
			factory.createArray<double>({ 3,1 }, { XInew[0], XInew[1], XInew[2] });
		TypedArray<double> m_t_GPS_new =
			factory.createArray<double>({ 1,1 }, { t_GPS_new });

		//Return the Matlab output variables
		outputs[0] = m_qNew; //[e0; e1; e2; e3](-1 to 1) estimated quaternions
		outputs[1] = m_VInew; //[VxI; VyI; VzI](m / s) estimated inertial frame velocities
		outputs[2] = m_XInew; //[xI; yI; zI](m) estimated inertial frame positions
		outputs[3] = m_t_GPS_new; //OUTPUT: [s] the most recent timestamp for new GPS data
	}
};