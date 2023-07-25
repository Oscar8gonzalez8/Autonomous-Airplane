%% import the data
[matlabFile,path] = uigetfile('*.csv', ...
    'Select The Flight Simulator data');
sim = readtable([path,matlabFile]);

%Time (s)
t = sim.Time_s;
dt_avg = mean(t(2:end)-t(1:end-1));
N = length(t);
%Gyro sensor data
xG = sim.xRate_rad_s; %(rad/s) x-axis gyrometer angular velocity
yG = sim.yRate_rad_s; %(rad/s) y-axis gyrometer angular velocity
zG = sim.zRate_rad_s; %(rad/s) z-axis gyrometer angular velocity
%Accelerometer sensor data
xA = sim.xAccel_m_s2; %(m/s2) x-axis accelerometer signal
yA = sim.yAccel_m_s2; %(m/s2) y-axis accelerometer signal
zA = sim.zAccel_m_s2; %(m/s2) z-axis accelerometer signal
%Magnetometer sensor data
xM = sim.xMag_uT; %(uT) x-axis magnetometer signal
yM = sim.yMag_uT; %(uT) y-axis magnetometer signal
zM = sim.zMag_uT; %(uT) z-axis magnetometer signal
%GPS sensor data
GPS_lat = sim.GPS_lat_deg; %(deg) GPS latitude
GPS_lon = sim.GPS_long_deg; %(deg) GPS longitude
GPS_Alt = sim.Alt_m; %(m) GPS altitude
%Actual quaternions (simulated)
e0 = sim.e0;
e1 = sim.e1;
e2 = sim.e2;
e3 = sim.e3;

