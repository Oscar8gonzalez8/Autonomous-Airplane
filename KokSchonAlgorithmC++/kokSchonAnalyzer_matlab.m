close all 
clear all 
clc 

[matlabFile, path] = uigetfile('*.csv', 'Select The PICO data');
picoData = readtable([path, matlabFile]);

[matlabFile, path] = uigetfile('*.csv', 'Select the Phone data');
phoneData = readtable([path, matlabFile]);

%[matlabFile, path] = uigetfile('*.mat', 'Select the Phone data');
%filePath = fullfile(path, matlabFile);
%phoneData = load(filePath);

% Pico data
% xyz magnetometer data
xM = picoData.Bx; 
yM = picoData.By; 
zM = picoData.Bz; 

% xyz accelerometer data 
xA = picoData.gFx; %x - axis 
yA = picoData.gFy;
zA = picoData.gFz;

% xyz gyrometer data
xG = picoData.wx;
yG = picoData.wy;
zG = picoData.wz;

% GPS latitude 
GPS_lat = picoData.Latitude; 
GPS_lon = picoData.Longitude; 
roll = picoData.Roll * 180 / pi; 
pitch = picoData.Pitch * 180 / pi; 
yaw = picoData.Azimuth * 180 / pi; 
[~, ind] = max(abs(yG));
t = picoData.time - picoData.time(ind);

%Phone data
xMi = phoneData.By; 
yMi = phoneData.Bx; 
zMi = -phoneData.Bz; 

xAi = -phoneData.gFy; 
yAi = -phoneData.gFx; 
zAi = phoneData.gFz;

xGi = phoneData.wy; 
yGi = phoneData.wx; 
zGi = -phoneData.wz; 

GPS_lati = phoneData.Latitude; 
GPS_loni = phoneData.Longitude; 
rolli = -phoneData.Roll; 
pitchi = -phoneData.Pitch; 
yawi = phoneData.Azimuth; 
[~, indI] = max(abs(yGi));
ti = phoneData.time - phoneData.time(ind);

figure
subplot(311)
plot(t, xM, ti, xMi)
title('Magnetometer')
ylabel('x')
grid on
subplot(312)
plot(t, yM, ti, yMi)
ylabel('y')
grid on
subplot(313)
plot(t, zM, ti, zMi)
ylabel('z')
xlabel('Time (s)')
grid on 
legend('Pico', 'Phone')


figure 
subplot(311)
plot(t, xA, ti, xAi)
title('Accelerometer')
ylabel('x')
grid on 
subplot(312)
plot(t, yA, ti, yAi)
ylabel('y')
grid on 
subplot(313)
plot(t, zA, ti, zAi)
ylabel('z')
xlabel('Time (s)')
grid on 
legend('Pico', 'Phone')

figure
subplot(311)
plot(t, xG, ti, xGi)
title('Gyrometer')
ylabel('x')
grid on 
subplot(312)
plot(t, yG, ti, yGi)
ylabel('y')
grid on 
subplot(313)
plot(t, zG, ti, zGi)
ylabel('z')
xlabel('Time (s)')

grid on 
legend('Pico', 'Phone')

figure
subplot(211)
plot(t, GPS_lat, ti, GPS_lati)
title('GPS')
ylabel('Latitude')
avgL = mean(GPS_lati(end-10:end)); 
ylim([avgL-0.001, avgL + 0.001])
grid on 
subplot(212)
plot(t, GPS_lon, ti, GPS_loni)
ylabel('Longitude')
avgL = mean(GPS_loni(end-10:end));
ylim([avgL-0.001, avgL+0.001])
xlabel('Time (s)')
grid  on 
legend('Pico', 'Phone')

%Orientation 
figure 
subplot(311)
plot(t, roll, ti, rolli)
title('Orientation')
ylabel('Roll')
grid on 
subplot(312)
plot(t, pitch, ti, pitchi)
ylabel('Pitch')
grid on 
subplot(313)
plot(t, yaw, ti, yawi)
ylabel('Yaw')

xlabel('Time (s)')
grid on 
legend('Pico', 'Phone')