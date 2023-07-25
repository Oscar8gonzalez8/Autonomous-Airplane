function q = function_QuaternionEstimator(qPrev, gyro, gravity, magnet,...
    dt, beta)
% q = function_QuaternionEstimator(qPrev, gyro, gravity, magnet, dt, beta)
%
% This function estimates the instantaneous value of the quaternion
% orientation.  It uses the previously estimated quaternion (qPrev),
% the gyrometer signal (gyro), the gravity signal (gravity), 
% the magnetometer signal (magnet), the timestep between estimates (dt)
% and other constants / signals. Your job is to create this function. 
% You must use all of these inputs, but you are not limited to these
% inputs only.  In fact, it may be necessary to use filtered gyro,
% gravityerometer, and magnetometer signals.
%
% INPUTS:
% qPrev: (0-1) 4x1 previous estimate of the quaternion [e0,e1,e2,e3]
% gyro: (rad/s) 3x1 3-axis gyrometer signal gyro = [gyro_x,gyro_y,gyro_z];
% gravity: (m/s2) 3x1 3-axis gravity signal...gravity is positive
%                                          ...acceleration is negative
% magnet: (uT) 3x1 3-axis magnetometer signal...assumes and inclination
%                                        ...angle of 67 degrees
% dt: (s) Timestep between quaternion estimates
% beta: (unitless) tuning parameter
%
% OUTPUTS:
% q: (0-1) 4x1 instantaneous estimate of the quaternion [e0,e1,e2,e3]
% 

%calculate the quaternions
q = qPrev; %Placeholder, you will need to change this.

