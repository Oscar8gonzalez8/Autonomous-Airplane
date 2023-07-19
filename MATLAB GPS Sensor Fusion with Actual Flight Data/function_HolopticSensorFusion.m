function [q,XI_new,VI_new, t_GPS_new,latitude_new,longitude_new,altitude_new] = ...
function_HolopticSensorFusion(...
q_old, XI_old, VI_old, lat0, lon0, alt0, t_GPS_old,...
gyro, accel, magnet,...
latitude_now, longitude_now, altitude_now, t_now, ...
GPS_available, dt, inc_angle_deg, alpha, beta, gamma)

latitude_new = latitude_now;
longitude_new = longitude_now;
altitude_new = altitude_now;

persistent aLP;
if isempty(aLP)
aLP = [0;0;0];
end

%Convert GPS and waypoint data to radians
phi = latitude_now * pi/180;
lambda = longitude_now * pi/180;
phi0 = lat0 * pi/180; %rad
lam0 = lon0 * pi/180; %rad

rEarth = 6371000; %m

%Get inertial displacements
xI = rEarth * (phi-phi0);
yI = get_yI(lambda, lam0, phi);
zI = -(altitude_now - alt0);
XI = [xI;yI;zI]; %m

xyDist = sqrt(xI^2+yI^2); %North-East distance traveled

if xyDist > 0.1
    %New GPS is available
    dtGPS = t_now - t_GPS_old; %(s) GPS timestep
    t_GPS_new = t_now; %(s) update the GPS time
    VI_new = (XI - XI_old)/dtGPS; %(m/s) inertial velocity
else
    t_GPS_new = t_GPS_old;
    VI_new = VI_old; %(m/s) inertial velocity
end

%Update XI_new
XI_new = XI;

RI2b = R_I2b(q_old);

%get the body-frame velocities
Vbody = [norm(VI_new);0;0];

%low-pass filter the accelerometer signal
tau = 0.02; %(s) low-pass filter time constant
aLP = (1-dt/max(dt,tau))*aLP + dt/max(dt,tau) * accel;%(m/s2)

%accelerometer's estimate of gravity
g_ak = aLP + cross(gyro, Vbody);
g_ak = g_ak / norm(g_ak);

%gyrometer's estimate of gravity
gGyro = RI2b * [0;0;1];

%Gyrometer's estimate of the geomagnetic vector
mGyro = RI2b * [cosd(inc_angle_deg); 0; sind(inc_angle_deg)];

%Magnetometer's measurement of geomagnetic vector
m = magnet / norm(magnet);

%Calculate the Kok Schon gradient
gradV = cross(gGyro, (g_ak-gGyro)) + alpha * cross(mGyro, (m-mGyro));

%Get the angular velocity update
dw = beta * gradV / norm(gradV);
size(dw)

%extract the quaternions
e0 = q_old(1);
e1 = q_old(2);
e2 = q_old(3);
e3 = q_old(4);

%Get the Sk matrix
Sk = [-e1,-e2,-e3;...
e0, -e3, e2;...
e3, e0, -e1;...
-e2, e1, e0];

qNew = q_old + dt/2*Sk*(gyro - dw);
q = qNew / norm(qNew);
end

function RI2b = R_I2b(q)
    e0 = q(1);
    e1 = q(2);
    e2 = q(3);
    e3 = q(4);
    RI2b = [e0^2+e1^2-e2^2-e3^2, 2*(e0*e3+e1*e2), 2*(e1*e3-e0*e2);...
    2*(e1*e2-e0*e3), e0^2-e1^2+e2^2-e3^2, 2*(e0*e1+e2*e3);...
    2*(e0*e2+e1*e3), 2*(e2*e3-e0*e1), e0^2-e1^2-e2^2+e3^2];
end

function yI = get_yI(lambda, lam0,phi)
    rEarth = 6371000; %m
    if abs(lambda-lam0) <= pi
        dlambda = lambda - lam0;
    else
        if lambda > lam0
            dlambda = lambda - (lam0 + 2*pi);
        else
            dlambda = (2*pi+lambda)-lam0;
        end
     end
        yI = sign(dlambda)*rEarth*acos((cos(dlambda)-1)*cos(phi)^2+1);
end
