function [delta_t,...[0 to 1] throttle command
    delta_e,...[-1 to 1] elevator command
    delta_a,...[-1 to 1] aileron command
    delta_r,...[-1 to 1] rudder command
    qNew,...[e0;e1;e2;e3] (-1 to 1) estimated quaternions
    VInew,...[VxI;VyI;VzI] (m/s) estimated inertial frame velocities
    XInew,...[xI;yI;zI] (m) estimated inertial frame positions
    wpIndex ...[0 to Nwaypoints] index of current waypoint target
    ]=NED_The_Exam1_Function( ...
        q,...[e0;e1;e2;e3] (-1 to 1) previous estimated quaternions
        VI,...[VxI;VyI;VzI] (m/s) previous estimated inertial frame velocities
        XI,...[xI;yI;zI] (m) previous estimated inertial frame positions
        gyro,...[wx;wy;wz] (rad/s) angular velocities
        accel,...[ax;ay;az] (m/s2) gravity positive, linear accelerations negative
        magnet,...[Mx;My;Mz] (uT) geomagnetic field strengths
        GPS_lat,...[lat] (deg) latitude angle
        GPS_lon,...[lon] (deg) longitude angle
        GPS_alt,...[-zI] (m) altitude 
        lat0,...[lat] (deg) initial latitude angle
        lon0,...[lon] (deg) initial longitude angle
        alt0,...[-zI] (m) initial altitude
        newGPS,...[0 or 1] flag indicating whether new GPS is available or not
        dt,...[dt] (s) iteration time step
        dtGPS, ...[dtGPS] (s) GPS time-step
        GPSwaypoints,...[lat1, lon1, alt1; lat2, lon2, alt2; lat3, lon3, alt3;...] (deg,deg,m) Nx3 Array of N GPS waypoints
        waypointIndex ...[0 to Nwaypoints] index of current waypoint target
    )

latitude_now = GPS_lat;
longitude_now = GPS_lon;
altitude_now = GPS_alt;
inc_angle_deg = 67;
XI_old = XI;
VI_old = VI;
alpha = 1; 
beta = 0.3;
gamma = 0.9;


% Extract the Quaternions 
e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);
[roll, pitch, yaw] = Quaternion2Euler(e0, e1, e2, e3);


for ii = 1:3
    if magnet(ii) == 0
        magnet(ii) = 0.0001;
    end
    if accel(ii) == 0
        accel(ii) = 0.0001;
    end
     if gyro(ii) == 0
        gyro(ii) = 0.0001;
    end
end

r = 6371000;
phiLat0 = lat0*(pi/180);
phiLat1 = latitude_now*(pi/180);
phiLon0 = lon0*(pi/180);
phiLon1 = longitude_now*(pi/180);

if newGPS
    xI = r*(phiLat1-phiLat0);
    if abs(phiLon1-phiLon0) <= pi
        deltaL = phiLon1-phiLon0;
    elseif abs(phiLon1-phiLon0) > pi
        if phiLon1 > phiLon0
            deltaL = phiLon1 - (phiLon0 + 2*pi);
        else
            deltaL = (2*pi+ phiLon1)-phiLon0;
        end
    end

yI = sign(deltaL)*r*acos((cos(deltaL)-1)*cos(phiLat1)^2+1);
zI = -(altitude_now-alt0);
XInew = [xI;yI;zI];

end

if newGPS
    V_GPS = (XInew - XI_old)/(dtGPS);
    VInew = (1-gamma)*VI_old+gamma*V_GPS;
else
    VInew = VI_old;
end

RI = [e0^2+e1^2-e2^2-e3^2,2*(e0*e3+e1*e2),2*(e1*e3-e0*e2);...
    2*(e1*e2-e0*e3),e0^2-e1^2+e2^2-e3^2,2*(e0*e1+e2*e3);...
    2*(e0*e2+e1*e3),2*(e2*e3-e0*e1),e0^2-e1^2-e2^2+e3^2];

VInew = RI*VInew;

if VInew(1) < 0.7*norm(VInew)
    VInew = [norm(VInew);0;0];
end

gak = accel+cross(gyro,VInew) / norm(accel+cross(gyro,VInew));
gk = RI*[0;0;1];
mk = RI*[cosd(inc_angle_deg);0;sind(inc_angle_deg)];
uM = magnet/norm(magnet);
dJ = cross(gk,gak-gk) + cross(alpha*mk,uM-mk);
dw = beta*(dJ/norm(dJ));
S = [-e1,-e2,-e3;...
    e0,-e3,e2;...
    e3,e0,-e1;...
    -e2,e1,e0];
uG = gyro/norm(gyro);
qG = q + dt/2 * S * (uG-dw);
%qG = function_RI2b_to_Quaternion(R);
qNew = qG/norm(qG);






%*************************** Start of GPS way point following 

lat = GPS_lat;
lon = GPS_lon; 
alt = GPS_alt; 


% The first three elements in each
% row are the inertial xI,d , yI,d , and zI,d coordinates of the waypoint. 
% If coordinates are specified as latitude (degrees), longitude (degrees), 
% and altitude (m) instead of displacements xI, yI , and zI 


% origin
waypoint = GPSwaypoints(1, 1:3); % Assuming GPSwaypoints returns a single output
lat0 = waypoint(1);
lon0 = waypoint(2);
alt0 = waypoint(3);

% desired way points 
waypoint_1 = GPSwaypoints(waypointIndex, :);
latd = waypoint_1(1);
lond = waypoint_1(2);
altd = waypoint_1(3);

% convert to latitude and longitude to radian 
phi = lat * pi/180;
lambda = lon * pi/180;
phi0 = lat0 * pi/180; %rad
lam0 = lon0 * pi/180; %rad
latd = latd * pi/180; 
lond = lond * pi/180; 

% Compare the actual positions with respect to the origin  
rEarth = 6371000; %m
%Get inertial displacements
xI = rEarth * (phi-phi0);
yI = get_yI(lambda, lam0, phi);
zI = -(alt - alt0);

% Compare the waypoints with respect to the origin of coord system
%Get inertial displacements
xId = rEarth * (latd-phi0);
yId = get_yI(lond, lam0, latd);
zId = -(altd - alt0);

delta_xI = xId - xI;  
delta_yI = yId - yI; 
delta_zI = zId - zI; 

absolute_dist = norm([delta_xI, delta_yI, delta_zI]);

if absolute_dist < 25
    wpIndex = waypointIndex + 1; 
else
    wpIndex = waypointIndex; 
end     

delta_r = 0; 
delta_t = 1; 
kp_roll_yaw = 0.41; 
% roll_desired = 0; 
kp_roll = 0.61;
kd_roll = 0.0684; 
% Calculated from xI, yI, and zI 
yaw_desired = atan2(delta_yI, delta_xI); 

% u5 is also ∆ψ aka yaw error, it uses the smallest angle difference
u5 = diff_in_angles(yaw_desired, yaw);

u4 = kp_roll_yaw * u5;

% roll_r or φx,r ∈ [−π/4,π/4] causes the airplane to bank to ±45
roll_r = max(-pi/4, min(u4, pi/4));

% u3 is also ∆φ aka roll error, it uses the smallest angle difference
u3 = diff_in_angles(roll_r, roll);

% The aileron command δa is a proportional-derivative (PD) controller 
% that depends on the roll error ∆φ and the roll-rate ωx 
u2 = kp_roll * u3; 

% only use the x-component for gyro?
u1 = u2 - (kd_roll * gyro(1));

delta_a = max(-1, min(u1, 1));

% The reference pitch angle θy,r depends on the desired pitch 
% angle θy,d , pitch trim θy,trim, and the square of the roll 
% error ∆φ^2

%The gain kp,φ,θ is a user-selected tuning parameter
kp_roll_pitch = 0.21; 

pitch_desired = asin(-delta_zI / absolute_dist);
theta_y_trim = 0; 
kp_pitch = -0.8; 

% The reference pitch angle θy,r depends on the desired pitch 
% angle θy,d , pitch trim θy,trim, and the square of the roll 
% error ∆φ^2.
theta_y_r = pitch_desired + theta_y_trim + kp_roll_pitch * u3^2;

% u6 is also the pitch error ∆θ
u6 = diff_in_angles(theta_y_r, pitch); 

u7 = kp_pitch*u6;

delta_e = max(-1, min(u7, 1));



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
    
    function delta_alpha = diff_in_angles(alpha2, alpha1)
        % This function calculates
        % the difference or error between two angles α1 and α2 
        delta = acos(cos(alpha1)*cos(alpha2) + sin(alpha1)*sin(alpha2));
        delta = abs(delta);
        if abs(alpha2 - alpha1) - delta > 0.1
            delta_alpha = -sign(alpha2 - alpha1) * abs(delta);
        else 
            delta_alpha = alpha2 - alpha1; 
        end
    end 



    %function to convert quaternions to euler angles
    function [roll,pitch,yaw] = Quaternion2Euler(e0,e1,e2,e3)
        roll = atan2(2*(e0.*e1+e2.*e3), (e0.^2+e3.^2-e1.^2-e2.^2));
        pitch = asin(max(-1,min(1,2*(e0.*e2-e1.*e3))));
        yaw = atan2(2*(e0.*e3+e1.*e2),(e0.^2+e1.^2-e2.^2-e3.^2));
    end


%*********************************NED_The_Final_Exam_Function
%*********************************NED_The_Final_Exam_Function
end
%*********************************NED_The_Final_Exam_Function
%*********************************NED_The_Final_Exam_Function

