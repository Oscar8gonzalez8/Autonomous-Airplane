%% import the data
[matlabFile,path] = uigetfile('*.csv', 'Select The Flight Data');
sim = readtable([path,matlabFile]); %Extract the variables

try
    %Time (s)
    t = sim.time; %(s) time vector
    dt_avg = mean(t(2:end)-t(1:end-1)); %(s) average time-step
    N = length(t);
catch 
    disp('Time data did not load')
end
try
    %Gyro sensor data
    xG = sim.wy; %(rad/s) x-axis gyrometer angular velocity
    yG = sim.wx; %(rad/s) y-axis gyrometer angular velocity
    zG = -sim.wz; %(rad/s) z-axis gyrometer angular velocity
catch
    disp('Gyro sensor data did not load')
end
try
    %Accelerometer sensor data
    xA = -sim.gFy; %(m/s2) x-axis accelerometer signal
    yA = -sim.gFx; %(m/s2) y-axis accelerometer signal
    zA = sim.gFz; %(m/s2) z-axis accelerometer signal
catch
    disp('Accelerometer data did not load')
end
try
    %Magnetometer sensor data
    xM = sim.By; %(uT) x-axis magnetometer signal
    yM = sim.Bx; %(uT) y-axis magnetometer signal
    zM = -sim.Bz; %(uT) z-axis magnetometer signal
catch
    disp('Magnetometer data did not load')
end
try
    %GPS sensor data
    GPS_lat = sim.Latitude; %(deg) GPS latitude
    GPS_lon = sim.Longitude; %(deg) GPS longitude
catch
    disp('GPS data did not load')
end
try
    %GPS speed data
    GPS_Speed = sim.Speed; %(m/s) GPS speed
catch
    disp('GPS speed did not load')
end
try
    %Barometric pressure (Altitude) data
    Altitude = (101325-sim.Altitude*100)/(1.8*9.81); %(m) GPS altitude
catch
    disp('Altitude data did not load')
end
try
    %Roll, Pitch, and Yaw angles
    Pitch = -sim.Pitch; %(deg) pitch angle
    Roll = -sim.Roll; %(deg) roll angle
    Yaw = sim.Azimuth; %(deg) yaw angle
    %Get the phone's predictions of the quaternions
    [e0,e1,e2,e3] = Euler2Quaternion(Roll, Pitch, Yaw);
catch
    disp('Inclinometer data did not load')
end

%Convert roll, pitch, and yaw angles in degrees to quaternions
function [e0,e1,e2,e3] = Euler2Quaternion(phi,theta,psi)
%Get cosines of half-angles
cs = cosd(psi/2);
ct = cosd(theta/2);
cp = cosd(phi/2);
%Get sines of half-angles
ss = sind(psi/2);
st = sind(theta/2);
sp = sind(phi/2);
%Get quaternions
e0 = cs.*ct.*cp+ss.*st.*sp;
e1 = cs.*ct.*sp-ss.*st.*cp;
e2 = cs.*st.*cp+ss.*ct.*sp;
e3 = ss.*ct.*cp-cs.*st.*sp;
end

