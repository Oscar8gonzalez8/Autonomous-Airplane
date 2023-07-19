function q = function_QuaternionEstimator(qPrev, gyro, accel, magnet,...
    dt, inc_angle_deg, betaG, betaM)
% q = function_QuaternionEstimator(qPrev, gyro, accel, magnet, dt)
%
% This function estimates the instantaneous value of the quaternion
% orientation.  It uses the previously estimated quaternion (qPrev),
% the gyrometer signal (gyro), the accelerometer signal (accel), 
% the magnetometer signal (magnet), the timestep between estimates (dt)
% and other constants / signals. Your job is to create this function. 
% You must use all of these inputs, but you are not limited to these
% inputs only.  In fact, it may be necessary to use filtered gyro,
% accelerometer, and magnetometer signals.
%
% INPUTS:
% qPrev: (0-1) previous estimate of the quaternion [e0,e1,e2,e3]
% gyro: (rad/s) 3-axis gyrometer signal gyro = [gyro_x,gyro_y,gyro_z];
% accel: (m/s2) 3-axis accelerometer signal...gravity is positive
%                                          ...acceleration is negative
% magnet: (uT) 3-axis magnetometer signal...assumes and inclination
%                                        ...angle of 67 degrees
% dt: (s) Timestep between quaternion estimates
% ...You will likely need to add more inputs.  Feel free to do so.
%
% OUTPUTS:
% q: (0-1) instantaneous estimate of the quaternion [e0,e1,e2,e3]
% ...You will likely need to add more outputs.  Feel free to do so.

%extract the quaternions
e0 = qPrev(1);
e1 = qPrev(2);
e2 = qPrev(3);
e3 = qPrev(4);

% Previous estimate of the quaternion 
S = [-e1, -e2, -e3;
      e0, -e3, e2; 
      e3, e0, -e1; 
     -e2, e1, e0];

% Euler integration 
qG = qPrev + (dt/2)*S*transpose(gyro); 

%extract the quaternions
e0 = qG(1);
e1 = qG(2);
e2 = qG(3);
e3 = qG(4);

% gryometer's estimate of the gravity unit vector 
gG = [2 * (e1*e3 - e0*e2);...
      2 * (e0*e1 + e2*e3);...
      e0^2 - e1^2 - e2^2 + e3^2];


% Geometric unit vector 
Rg = [e0^2 + e1^2 - e2^2 - e3^2, 2*(e0*e3 + e1*e2), 2*(e1*e3 - e0*e2);...
      2*(e1*e2 - e0*e3), e0^2 - e1^2 + e2^2 - e3^2, 2*(e0*e1 + e2*e3);...
      2*(e0*e2 + e1*e3), 2*(e2*e3 - e0*e1), e0^2-e1^2-e2^2+e3^2];

Inert = [cosd(inc_angle_deg);...
         0;...
         sind(inc_angle_deg)];

Mg = Rg*Inert; 

% gytrometer's estiate of gravity unit vector at present time 
% geomatric unit vector 
% both are stacked to geother to get measurement prediction
yG = [gG; Mg];

% estimate of the gravity unit vector 
accelT = transpose(accel);
ga = accelT/norm(accelT);

% the magnetometer measurement of the geomagnetic
% unit vector
magnetT = transpose(magnet);
uM = magnetT / norm(magnetT);

%  measurement vector 
yk = [ga ; uM];


% The filter coefficient βj, j = g or j = m decides 
% whether to trust the gyrometer estimate yˆG,k

% The complementary filter 
BetaMatrix = [betaG*eye(3,3), zeros(3, 3);...
              zeros(3,3), betaM*eye(3,3)];
yc = (eye(6,6) - BetaMatrix) * yG + BetaMatrix * yk; 

ug = yc(1:3);
um = yc(4:end);

ue = cross(ug, um)/norm(cross(ug, um));
un = cross(ue, ug)/norm(cross(ue, ug));

R = [un, ue, ug];

% Quaternion Orientation from a Rotation Matrix
q = function_RI2b_to_Quaternion(R);

% estimate is forced to be a unit quaternion by
% normalizing at each iteration of the algorithm
q = q / norm(q);

end 
