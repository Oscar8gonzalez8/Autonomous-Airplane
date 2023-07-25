close all
clear all
clc

%% import the simulator data
ImportSimulatorData
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
qTrue = [e0,e1,e2,e3]; %(0-1) simulated quaternion orientation

N = length(t);

%Estimate the quaternions and compare with the true values
qEst = zeros(4,N);
qEst(:,1) = [1;0;0;0];
figure
hold on
for ii = 2:N
    %get the time-constant and filter coefficient
    dt = t(ii) - t(ii-1);

    %Calculate an estimate of gravity
    gravity = Accel(ii,:)';
    
    %store the estimated quaternion
    qEst(:,ii) = function_QuaternionEstimator(...
        qEst(:,ii-1),gyro(ii,:)',gravity,Magnet(ii,:)',dt);
    
    if ~mod(ii,4) % change this value to run faster or slower
        clf('reset')
        DrawAirplane(0,0,0,qTrue(ii,:))
        DrawAirplane_ghost(0,0,0,qEst(:,ii)')
        drawnow
    end
end

% Plot the trajectory of the quaternion orientation
figure, 
subplot(411)
plot(t,qEst(1,:), t, e0)
subplot(412)
plot(t,qEst(2,:), t, e1)
subplot(413)
plot(t,qEst(3,:), t, e2)
subplot(414)
plot(t,qEst(4,:), t, e3)


