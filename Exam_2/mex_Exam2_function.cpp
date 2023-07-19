//Include the Matlab Executable (mex) header files to use the following:
//mex, MexFunction, matlab::mex::Function, matlab::data,
//matlab::mex::ArgumentList, getNumberOfElements(), MATLABEngine(),
//TypedArray, ArrayFactory
#include "mex.hpp"
#include "mexAdapter.hpp"
#include "HolopticSensorFusion.h"
#include "AutoControl.h"


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
		bool newGPS; //[0 or 1] flag indicating whether new GPS is available or not
		double dt; //[dt](s) iteration time step
		double dtGPS; //[dtGPS](s) GPS time step
		double GPSwaypoints[5][3]; //[lat1, lon1, alt1; lat2, lon2, alt2; lat3, lon3, alt3; ...](deg, deg, m) Nx3 Array of N GPS waypoints
		double waypointIndex; //[0 to Nwaypoints] index of current waypoint target

		//Output variables
		double delta_t = 0.0; //[0 to 1] throttle command
		double delta_e = 0.0; //[-1 to 1] elevator command
		double delta_a = 0.0; //[-1 to 1] aileron command
		double delta_r = 0.0; //[-1 to 1] rudder command
		double qNew[4] = { 0.0 }; //[e0; e1; e2; e3](-1 to 1) estimated quaternions
		qNew[0] = 1.0;
		double VInew[3] = { 0.0 }; //[VxI; VyI; VzI](m / s) estimated inertial frame velocities
		double XInew[3] = { 0.0 }; //[xI; yI; zI](m) estimated inertial frame positions
		double wpIndex = 0.0; //[0 to Nwaypoints] index of current waypoint target

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
		TypedArray<double> m_newGPS = std::move(inputs[12]);
		TypedArray<double> m_dt = std::move(inputs[13]);
		TypedArray<double> m_dtGPS = std::move(inputs[14]);
		TypedArray<double> m_GPSwaypoints = std::move(inputs[15]);
		TypedArray<double> m_waypointIndex = std::move(inputs[16]);

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
		newGPS = (bool)m_newGPS[0];
		dt = m_dt[0];
		dtGPS = m_dtGPS[0];
		waypointIndex = m_waypointIndex[0];

		//Since GPSwaypoints is a matrix, it is converted differently
		//Put the elements from the OutMat matrix into the Out_Array
		int ii = 0; //Row index
		int jj = 0; //Column index
		for (auto& elem : m_GPSwaypoints) {
			GPSwaypoints[ii][jj] = elem; //Populate the A matrix
			ii++;
			if (ii > 5 - 1) {
				//Go to the next column
				ii = 0;
				jj++;
			}
		}

		//
		//Run the Holoptic sensor fusion algorithm
		//runHolopticSensorFusion(
		//	qNew, //OUTPUT: updated quaternion orientation
		//	Vbody, //OUTPUT: [m/s] updated body-frame velocities estimate
		//	VInew, //OUTPUT: [m/s] updated inertial-frame velocities estimate
		//	XInew, //OUTPUT: [m] updated inertial positions estimate
		//	q, //INPUT: previous quaternion orientation
		//	VI, //INPUT: [m/s] previous estimate of inertial-frame velocities
		//	XI, //INPUT: [m] previous estimate of inertial-frame positions
		//	gyro, //INPUT: [rad/s] 3-axis gyro data with bias removed
		//	accel, //INPUT: [m/s2] x-front, y-right, z-down raw accelerometer signal, bias removed
		//	magnet, //INPUT: [any] x-front, y-right, z-down calibrated magnetometer signal any units, bias removed
		//	GPS_lat, //INPUT: [degrees] GPS latitude
		//	GPS_lon, //INPUT: [degrees] GPS longitude
		//	GPS_alt, //INPUT: [m] GPS altitude or barometric pressure altitude
		//	lat0, //INPUT: [degrees] latitude at the origin [0,0,0]
		//	lon0, //INPUT: [degrees] longitude at the origin [0,0,0]
		//	alt0, //INPUT: [m] altitude at the origin [0,0,0]
		//	67.0, //INPUT: [degrees] the geomagnetic inclination angle
		//	dt, //INPUT: [s] time-step
		//	dtGPS, //INPUT: [s] time-step of GPS/altitude measurements
		//	0.1, //INPUT: [<< 1] small-valued tuning parameter 
		//	1.0, //INPUT: [1,2] tuning parameter for magnetometer trust weighting
		//	dt/1.0, //INPUT: [0,1] low-pass filter constant for GPS velocity
		//	0.7, //INPUT: [0.5,0.9] fraction of total velocity in the forward direction
		//	newGPS //INPUT: [true or false] flag to indicate when GPS data is available
		//);

		double Vbody[3];
		// Run the Holoptic Sensor Fusion Algorithm
		runHolopticSensorFusion(
			qNew,         // OUTPUT: updated quaternion orientation
			Vbody,             // OUTPUT: XI_new [m/s] updated body-frame velocities estimate
			VInew,         // OUTPUT: VI_new [m/s] updated inertial-frame velocities estimate
			XInew,          // OUTPUT: lat_new, long_new, alt_new [m] updated inertial positions estimate
			q,             // INPUT: previous quaternion orientation
			VI,     // INPUT: [m/s] previous estimate of inertial-frame velocities
			XI,      // INPUT: [m] previous estimate of inertial-frame positions
			gyro,              // INPUT: [rad/s] 3-axis gyro data with bias removed
			accel,          // INPUT: [m/s2] x-front, y-right, z-down raw accelerometer signal, bias removed
			magnet,           // INPUT: [any] x-front, y-right, z-down calibrated magnetometer signal, any units, bias removed
			GPS_lat,           // INPUT: [degrees] GPS latitude
			GPS_lon,          // INPUT: [degrees] GPS longitude
			GPS_alt,               // INPUT: [m] GPS altitude or barometric pressure altitude
			lat0,    // INPUT: [degrees] latitude at the origin [0,0,0]
			lon0,   // INPUT: [degrees] longitude at the origin [0,0,0]
			alt0,        // INPUT: [m] altitude at the origin [0,0,0]
			67.0,// INPUT: [degrees] the geomagnetic inclination angle
			dt,                     // INPUT: [s] time-step
			dtGPS, //INPUT: [s] time-step of GPS/altitude measurements
			0.3,                  // INPUT: [<< 1] small-valued tuning parameter
			1.0,                   // INPUT: [1,2] tuning parameter for magnetometer trust weighting			
			dt / 1.0,                  // INPUT: [0,1] low-pass filter constant for GPS velocity
			0.7,                // INPUT: [0.5,0.9] fraction of total velocity in the forward direction
			newGPS //INPUT: [true or false] flag to indicate when GPS data is available
		);

		//Perform the autonomous flight control
		AutoControl(
			&delta_t,
			&delta_e,
			&delta_a,
			&delta_r,
			&wpIndex,
			qNew,
			gyro,
			waypointIndex,
			GPSwaypoints,
			GPS_lat,
			GPS_lon,
			GPS_alt,
			lat0,
			lon0,
			alt0
		);

		//Create the output Matlab array
		ArrayFactory factory;

		//Create the Matlab output variables
		TypedArray<double> m_delta_t = 
			factory.createArray<double>({ 1,1 }, { delta_t });
		TypedArray<double> m_delta_e =
			factory.createArray<double>({ 1,1 }, { delta_e });
		TypedArray<double> m_delta_a =
			factory.createArray<double>({ 1,1 }, { delta_a });
		TypedArray<double> m_delta_r =
			factory.createArray<double>({ 1,1 }, { delta_r });
		TypedArray<double> m_qNew =
			factory.createArray<double>({ 4,1 }, { qNew[0], qNew[1], qNew[2], qNew[3] });
		TypedArray<double> m_VInew =
			factory.createArray<double>({ 3,1 }, { VInew[0], VInew[1], VInew[2] });
		TypedArray<double> m_XInew =
			factory.createArray<double>({ 3,1 }, { XInew[0], XInew[1], XInew[2] });
		TypedArray<double> m_wpIndex =
			factory.createArray<double>({ 1,1 }, { wpIndex });

		//Return the Matlab output variables
		outputs[0] = m_delta_t; //[0 to 1] throttle command
		outputs[1] = m_delta_e; //[-1 to 1] elevator command
		outputs[2] = m_delta_a; //[-1 to 1] aileron command
		outputs[3] = m_delta_r; //[-1 to 1] rudder command
		outputs[4] = m_qNew; //[e0; e1; e2; e3](-1 to 1) estimated quaternions
		outputs[5] = m_VInew; //[VxI; VyI; VzI](m / s) estimated inertial frame velocities
		outputs[6] = m_XInew; //[xI; yI; zI](m) estimated inertial frame positions
		outputs[7] = m_wpIndex; //[0 to Nwaypoints] index of current waypoint target
	}
};


