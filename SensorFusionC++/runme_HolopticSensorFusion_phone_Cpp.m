close all
clear all
clc

%% compile the C++ code
mex -g mex_HolopticSensorFusion.cpp HolopticSensorFusion_v1.cpp

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
qTrue = [e0,e1,e2,e3]; %(0-1) simulated quaternion orientation

%Get the length of the time vector
N = length(t);

%Estimate the quaternions and compare with the true values
qEst = zeros(4,N);
qEst(:,1) = [1;0;0;0];
XI = zeros(3,N);
VI = zeros(3,N);
GPS_available = 1;
inc_angle_deg = 67;
alpha = 1;
beta = 0.3;
gamma = 0.9;
fraction = 0.7;
t_GPS = 0;
Lat0 = GPS_lat(1);
Lon0 = GPS_lon(1);
Alt0 = Altitude(1);

%Convert the GPS path to xI and yI displacements
[pathX,pathY] = getXYpath(GPS_lat,GPS_lon, Lat0, Lon0);

figure('WindowState','maximized')
hold on
for ii = 2:N
    %get the time-constant and filter coefficient
    dt = t(ii) - t(ii-1);

    %estimate orientation, displacement, and speed
    [qEst(:,ii),VI(:,ii),XI(:,ii),t_GPS] = ...
        mex_HolopticSensorFusion(...
            qEst(:,ii-1), VI(:,ii-1), XI(:,ii-1), ...
            gyro(ii,:)', Accel(ii,:)', Magnet(ii,:)',...
            GPS_lat(ii), GPS_lon(ii), Altitude(ii), ...
            Lat0,Lon0,Alt0,t(ii), t_GPS, dt, ...
            GPS_available, inc_angle_deg, ...
            beta, alpha, gamma, fraction);
    
    if ~mod(ii,200) % change this value to run faster or slower
        clf('reset')
        subplot(121)
        % hold on
        % DrawAirplane(0,0,0,qTrue(ii,:))
        DrawAirplane_ghost(0,0,0,qEst(:,ii)')
        subplot(122)
        plot(pathX,-pathY, XI(1,ii),-XI(2,ii),'o')
        axis('equal')
        xlabel('North')
        ylabel('West')
        drawnow
    end
end

figure,  scatter3(XI(1,:),-XI(2,:),-XI(3,:)) %show the 3D path of the airplane
xlabel('North')
ylabel('West')
zlabel('Up')
grid on

%function to convert quaternions to euler angles
function [roll,pitch,yaw] = Quaternion2Euler(e0,e1,e2,e3)
roll = atan2(2*(e0.*e1+e2.*e3), (e0.^2+e3.^2-e1.^2-e2.^2));
pitch = asin(max(-1,min(1,2*(e0.*e2-e1.*e3))));
yaw = atan2(2*(e0.*e3+e1.*e2),(e0.^2+e1.^2-e2.^2-e3.^2));
end

function [pathX,pathY] = getXYpath(GPS_lat,GPS_lon, Lat0, Lon0)
% [pathX,pathY] = getXYpath(GPS_lat,GPS_lon, Lat0, Lon0);
N = length(GPS_lat);
%Allocate memory to store the GPS path of the plane
pathX = zeros(1,N); 
pathY = pathX;
for ii = 1:N
    % calculate displacement from GPS
    dLambda = (GPS_lon(ii) - Lon0) * pi/180;
    if abs(dLambda) <= pi
        dLongitude = dLambda;
    elseif abs(dLambda) > pi
        if GPS_lon(ii) > Lon0
            dLongitude = (GPS_lon(ii) - (Lon0+360))*pi/180;
        else
            dLongitude = ((360+GPS_lon(ii)) - Lon0)*pi/180;
        end
    end
    rEarth = 6371000; %(m) Earth's radius
    %Get North xI displacement (m)
    xI = rEarth * (GPS_lat(ii) - Lat0)*pi/180;
    %Get East yI displacement (m)
    yI = sign(dLongitude)*rEarth*...
        acos((cos(dLongitude)-1)*(cosd(GPS_lat(ii)))^2+1);
    %Store the path
    pathX(ii) = xI;
    pathY(ii) = yI;
end
end