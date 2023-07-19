close all
clear all
clc

%% import the phone data
ImportDataFromPhone
%The data that is imported is
%t: Time (s)
%dt_avg: Average Time Step (s)
%xG:(rad/s) x-axis gyrometer angular velocity
%yG:(rad/s) y-axis gyrometer angular velocity
%zG:(rad/s) z-axis gyrometer angular velocity
%xA:(m/s2) x-axis accelerometer signal
%yA:(m/s2) y-axis accelerometer signal
%zA:(m/s2) z-axis accelerometer signal
%xM:(uT) x-axis magnetometer signal
%yM:(uT) y-axis magnetometer signal
%zM:(uT) z-axis magnetometer signal
%e0:scalar part of the quaternion
%e1:x-axis part of the quaternion
%e2:x-axis part of the quaternion
%e3:x-axis part of the quaternion
gyro = [xG,yG,zG]; %(rad/s) 3-axis gyrometer data
Accel = [xA,yA,zA]; %(m/s2) 3-axis accelerometer data
Magnet = [xM,yM,zM]; %(uT) 3-axis magnetometer data

%Get the length of the time vector
N = length(t);

%Estimate the quaternions and compare with the true values
qEst = zeros(4,N);
qEst(:,1) = [1;0;0;0];
figure
hold on
for ii = 2:N
    %get the time-constant and filter coefficient
    dt = t(ii) - t(ii-1);
    
    %store the estimated quaternion
    qEst(:,ii) = function_QuaternionEstimator(...
        qEst(:,ii-1),gyro(ii,:)',Accel(ii,:)',Magnet(ii,:)',dt,...
        67, 1.2, 0.1);
    
    if ~mod(ii,20) % change this value to run faster or slower
        clf('reset')
        DrawAirplane(0,0,0,qEst(:,ii)')
        drawnow
    end
end
