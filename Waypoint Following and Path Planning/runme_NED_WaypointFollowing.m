%% This is the runme file for NED, the MATLAB flight simulator
% This version is adapted for GPS waypoint tracking
% Originally created 14 July, 2022, for ME 410
% Modified 22 November 2022
% For derivations and notation, refer to the ME 410 book,
% "Autonomous Flight in Unmanned Airplanes" by Bro. Pence 

%Reset everything
close all
clear all
clc

%% set up the airplane parameters
%GPS waypoints lat (deg), lon (deg), alt (m)
GPSwaypoints = [43.80960,-111.78573, 20;...
                43.80510,-111.78635, 300;...
                43.81230,-111.79570, 450;...
                43.81230,-111.77950, 150;...
                43.80960,-111.78573, 3];
waypointIndex = 1; %(-) index of current waypoint
lat0 = 43.80960; %(deg) initial latitude
lon0 = -111.78573; %(deg) initial longitude

%% Set the initial conditions for the airplane state and wind gust state
% The airplane state is
% x=[u;v;w;p;q;r;xI;yI; zI;    e0;e1;    e2; e3]
x = [2;0;0;0;0;0; 0; 0;-20;0.9763; 0;0.2164; 0];

% The wind gust state is xGust = [uwg;x1;x2;x3;x4;Va]
% Note: Va is the relative airspeed of the plane from the previous
% iteration
xGust = [0;0;0;0;0;1];

%% set the airplane constants in a structure named c
c.mass = 1; %[kg] airplane mass
c.Jxx = 0.12; %[kg m2] Moment of inertia about x-axis (nose)
c.Jyy = 0.2; %[kg m2] Moment of inertia about y-axis (right-wing)
c.Jzz = 0.18; %[kg m2] Moment of inertia about z-axis (down)
c.Jxz = 0.015; %[kg m2] xz Product of inertia
c.rho = 1.27; %[kg/m3] air density
c.S = 1.5; %[m] wing span
c.c = 0.3; %[m] wing chord
c.A = c.S*c.c; %[m2] wing area
c.CL0 = -0.9;
c.CL1 = 2.5;
c.CL2 = 1;
c.CL3 = 11;
c.CL4 = 0.004;
c.CL_delta_e = 0.1; %[-] Elevator lift coefficient
c.CD0 = 0.06;
c.CD2 = 0.44;
c.CD_delta_e = 0.001; %[-] Elevator drag coefficient
c.Cy_beta = 0.015; %[-] Sideslip coefficient
c.Cy_delta_r = 0.001; %[-] Rudder sideslip coefficient
c.CTy_alpha = 0.15; %[-] Angle-of-attack pitch coefficient
c.CTy_delta_e = 0.1; %[-] Elevator pitch coefficient
c.bq = 0.5; %[N m s] Resistive pitch torque coefficient
c.C_delta_a = 0.03; %[-] Aileron roll torque coefficient
c.bp = 1; %[N m s] Resistive roll torque coefficient
c.CTz_beta = 0.005; %[-] Sideslip yaw torque coefficient
c.CTz_delta_r = 0.04; %[-] Rudder yaw torque coefficient
c.br = 0.5; %[N m s] Resistive yaw torque coefficient
c.n_max = 170; %[rev/s] maximum propeller speed
c.D = 10*0.0254; %[m] propeller diameter
c.alpha_b = 5; %[inch] propeller pitch

%% Set the wind model constants in a structure named wind
wind.ThereIsWind = 1; %[0 or 1] flag to turn wind on (1) or off (0)
wind.uw_North = -1; %[m/s] constant wind speed from south to north
wind.vw_East = 0.6; %[m/s] constant wind speed from west to east
wind.ww_Down = -0.1; %[m/s] constant downward wind speed
wind.Lu = 200; %[m] x-axis gust parameter
wind.Lv = wind.Lu; %[m] y-axis gust parameter
wind.Lw = 50; %[m] z-axis gust parameter
wind.sigma_u = 1.06; %[m/s] x-axis gust parameter
wind.sigma_v = wind.sigma_u; %[m/s] y-axis gust parameter
wind.sigma_w = 0.7; %[m/s] z-axis gust parameter

%% set other constants, initialize arrays
dt = 0.014;
t = []; %[s] time array
u = []; %[m/s] forward velocity
xI = []; yI = []; zI = []; %[m] inertial positions
da = []; 
de = [];
delta_t = 0.8; %initial throttle position
ii = 0; %initialize the iteration counter

%% Run the simulation
h_fig = figure('WindowState','maximized'); hold on
while x(9) < 0, ii = ii + 1;
    %Store the outputs
    xI = [xI;x(7)];
    yI = [yI;x(8)];
    zI = [zI;x(9)];
    t = [t;ii*dt];
    u = [u;x(1)];

    % get the function inputs
    %get the Euler angles
    [roll,pitch,yaw] = Quaternion2Euler(x(10:13));
    gyro = x(4:6);
    q = x(10:end); %(-1 to 1) normalized quaternion orientation
    [lat,lon] = getLatitudeLongitude(x(7), x(8), lat0, lon0);
    alt = -x(9);

    %Get the user input commands
    [delta_t, delta_e, delta_a, delta_r, wpIndex] = ...
        GPS_WaypointFollowingFunction( ...
            GPSwaypoints,...[lat1, lon1, alt1; lat2, lon2, alt2; lat3, lon3, alt3;...] (deg,deg,m) Nx3 Array of N GPS waypoints
            waypointIndex, ...[0 to Nwaypoints] index of current waypoint target
            roll, ...(rad) roll angle
            pitch, ...(rad) pitch angle
            yaw, ...(rad) yaw angle
            gyro, ...[p;q;r] (rad/s) vector of gyro measurements
            q, ...[e0;e1;e2;e3] (-1 to 1) quaternion orientation
            lat, ...(deg) latitude
            lon, ...(deg) longitude
            alt ...(m) altitude
    );

    da(ii) = delta_a; 
    de(ii) = delta_e;
    waypointIndex = wpIndex;

    %If all the waypoints have been reached, stop the simulation
    if wpIndex > height(GPSwaypoints)
        break;
    end

    if wind.ThereIsWind
        %Get the wind gusts and North-East-Down constant wind
        uvw_gust = WindGustOutputs(xGust,wind);
        ned_wind = [wind.uw_North; wind.vw_East; wind.ww_Down];
    
        %Calculate the airspeeds and wind-frame parameters
        [Va, alpha, beta, ur] = AirspeedVariables(x(1:3), ...
            ned_wind, ...
            uvw_gust, ...
            x(10:13));
    else
        %Calculate the airspeeds and wind-frame parameters
        [Va, alpha, beta, ur] = AirspeedVariables(x(1:3), ...
            [0;0;0], ...
            [0;0;0], ...
            x(10:13));
    end

    %Calculate the gravity forces
    [fxg,fyg,fzg] = gravityForces(c.mass,x(10:13));

    %Calculate the propeller forces and torques
    [fxT, TxQ] = PropellerThrustAndTorque(delta_t,ur,c);

    %Calculate the aerodynamic forces and torques
    [fxAero,fyAero,fzAero,TxAero,TyAero,TzAero] = ...
        AeroForcesAndTorques(x,Va,alpha,beta,...
        delta_e,delta_a,delta_r,c);

    %Combine all forces and torques
    fx = fxAero + fxT + fxg;
    fy = fyAero + fyg;
    fz = fzAero + fzg;
    Tx = TxAero + TxQ;
    Ty = TyAero;
    Tz = TzAero;

    %Use Runge-Kutta to calculate the airplane motion
    k1 = AirplaneDynamics(x, fx, fy, fz, Tx, Ty, Tz, c);
    k2 = AirplaneDynamics(x+dt*k1, fx, fy, fz, Tx, Ty, Tz, c);
    x = x+dt/2*(k1+k2);

    %Normalize the quaternions
    qNorm = norm(x(10:13));
    x(10:13) = x(10:13) / qNorm;

    %Use Euler to calculate the gust dynamics
    k1g = GustDynamics(xGust,wind);
    xGust(1:end-1) = xGust(1:end-1) + dt*k1g;
    xGust(end) = Va;

    if ~mod(ii,30)
        clf(h_fig)
        hold on
        DrawAirplane(x(7),x(8),x(9),x(10),x(11),x(12),x(13))
        drawnow
    end
end

% plot the outputs
wayPoints = [-500,-50,-300;...
    300,-800,-450;...
    300,500,-150;...
    0,0,0];
figure, hold on, scatter3(xI,yI,-zI) %show the 3D path of the airplane
scatter3(wayPoints(:,1),wayPoints(:,2),-wayPoints(:,3),300)
xlabel('North')
ylabel('East')
zlabel('Up')
grid on

figure, plot(t, u*2.237) %Show the forward speed of the airplane
xlabel('Time (s)')
ylabel('Forward Speed [mph]')
grid on


figure, plot(t, da, 'b')
title("delta a graph")
xlabel('Time (s)')
ylabel('delta a')
hold on 
figure
plot(t, de, 'r')
xlabel('Time (s)')
ylabel('delta e')
hold off
grid on 
%% This function calculates the Euler angles
function [phi,theta,psi] = Quaternion2Euler(q)
e0=q(1);
e1=q(2);
e2=q(3);
e3=q(4);
phi = atan2(2*(e0*e1+e2*e3), (e0^2+e3^2-e1^2-e2^2));
theta = asin(max(-1,min(1,2*(e0*e2-e1*e3))));
psi = atan2(2*(e0*e3+e1*e2),(e0^2+e1^2-e2^2-e3^2));
end

%% This function calculates latitude and longitude
function [lat,lon] = getLatitudeLongitude(dN, dE, lat0, lon0)
rEarth = 6371e3; %(m) earth's radius
lat = lat0 + dN / (rEarth*pi/180); %(deg) latitude
lon = lon0 + sign(dE)*acosd(1+(cos(dE/rEarth)-1)/cosd(lat0)^2);%(deg) longitude
end

%% This function calculates the wind gust outputs
function uvw_gust = WindGustOutputs(xGust,wind)
Va = xGust(end);
Cvw = [Va/sqrt(3*wind.Lv), 1];
Cww = [Va/sqrt(3*wind.Lw), 1];
uvw_gust = [xGust(1);Cvw*xGust(2:3);Cww*xGust(4:5)];
end

%% This function calculates the inertial-to-body frame rotation matrix
function RI2b = R_I2b(q)

%Extract the components of the quaternion q
e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);

%Get the rotation matrix
RI2b = [e0^2+e1^2-e2^2-e3^2, 2*(e0*e3-e1*e2), 2*(e1*e3-e0*e2);...
    2*(e1*e2-e0*e3), e0^2-e1^2+e2^2-e3^2, 2*(e0*e1+e2*e3);...
    2*(e0*e2+e1*e3), 2*(e2*e3-e0*e1), e0^2-e1^2-e2^2+e3^2];
end

%% This function calculates Va, alpha, and beta
function [Va, alpha, beta, ur] = AirspeedVariables(...
    uvw, ned_wind, uvw_gust, q)

%get the Inertial-to-body frame rotation matrix
RI2b = R_I2b(q);

%get the wind velocities in the body-frame
uvw_wind = RI2b*ned_wind + uvw_gust;

%get the relative velocities
uvw_r = uvw - uvw_wind;

%Magnitude of the relative airspeed
Va = sqrt(uvw_r(1)^2+uvw_r(2)^2+uvw_r(3)^2);

%alpha and beta
if Va > 0.1
    alpha = atan(uvw_r(3)/Va);
    beta = asin(max(-1,min(1,uvw_r(2)/Va)));
else
    alpha = 0;
    beta = 0;
end

%Get the relative forward velocity
ur = uvw_r(1);
end

%% This function calculates the gravity forces in the body frame
function [fxg,fyg,fzg] = gravityForces(m,q)
% [fxg,fyg,fzg] = gravityForces(m,q)
%
%OUTPUTs:
% fxg: (N) gravity force in the body-fixed x-direction
% fyg: (N) gravity force in the body-fixed y-direction
% fzg: (N) gravity force in the body-fixed z-direction
%
%INPUTS:
% m: (kg) airplane mass
% e0: (unitless) quaternion
% e1: (unitless) quaternion
% e2: (unitless) quaternion
% e3: (unitless) quaternion

%extract the quaternions
e0 = q(1);
e1 = q(2);
e2 = q(3);
e3 = q(4);

g = 9.81; %(m/s2) gravitational acceleration

mg = m*g; %(N) gravitational force

F = mg*[2*(e1*e3-e0*e2);...
    2*(e0*e1+e2*e3);...
    e0^2-e1^2-e2^2+e3^2];

%Gravity force components
fxg = F(1);
fyg = F(2);
fzg = F(3);
end

%% This function calculates the propeller thrust and torque
function [fxT, TxQ] = PropellerThrustAndTorque(delta_t,ur,c)
%[fxT, TxQ] = PropellerThrustAndTorque(delta_t,ur,c)
%
%OUTPUTS:
%fxT: (N) propeller thrust in the body-fixed x-axis
%TxQ: (N m) propeller torque around the body-fixed x-axis
%
%INPUTS:
%delta_t: (0-1) throttle command
%ur: (m/s) relative airspeed in the body-fixed x-direction
%c: A structure with at least the following items
%  c.n_max: (rev/s) Maximum propeller speed
%  c.D: (m) propeller diameter
%  c.alpha_b: (inch) propeller pitch
%  c.rho: (kg/m3) air density

n = c.n_max * (1-(1-delta_t)^2);

if n < 0.1
    %motor is off
    fxT = 0;
    TxQ = 0;
else
    J = ur / n / c.D;
    %prop torque
    zQ = 1/(1+exp(10*(0.16+0.05*c.alpha_b-J)));
    CQL = 0.0121-0.017*(J-0.1*(c.alpha_b-5));
    CQu = 0.0057+0.00125*(c.alpha_b-5)-0.0005*J;
    CQ = zQ*CQL+(1-zQ)*CQu;
    TxQ = c.rho*n^2*c.D^5*CQ;
    %prop thrust
    z = 1/(1+exp(20*(0.14+0.018*c.alpha_b-0.6*J)));
    CTL = 0.11-0.18*(J-0.105*(c.alpha_b-5));
    CTu = 0.1+0.009*(c.alpha_b-5)-0.065*J;
    CT = z*CTL+(1-z)*CTu;
    fxT = c.rho*n^2*c.D^4*CT;
end
end

%% This function calculates the aerodynamic forces and torques
function [fxAero,fyAero,fzAero,TxAero,TyAero,TzAero] = ...
    AeroForcesAndTorques(x,Va,alpha,beta,...
    delta_e,delta_a,delta_r,c)
%[fxAero,fyAero,fzAero,TxAero,TyAero,TzAero] = ...
%    AeroForcesAndTorques(x,Va,alpha,beta,...
%    delta_e,delta_a,delta_r,c)
%
%OUTPUTS:
% fxAero: (N) aerodynamic force in the body-fixed x-direction
% fyAero: (N) aerodynamic force in the body-fixed y-direction
% fzAero: (N) aerodynamic force in the body-fixed z-direction
% TxAero: (N m) aerodynamic torque around the body-fixed x-axis
% TyAero: (N m) aerodynamic torque around the body-fixed y-axis
% TzAero: (N m) aerodynamic torque around the body-fixed z-axis
%
%INPUTS:
% x: a vector of 13 state variables x=[u;v;w;p;q;r;xI;yI;zI;e0;e1;e2;e3]
% Va: (m/s) airspeed
% alpha: (rad) angle of attack
% beta: (rad) side-slip angle
% delta_e: (-1 to 1) elevator command
% delta_a: (-1 to 1) aileron command
% delta_r: (-1 to 1) rudder command
% c: a struct with constants

faero = 0.5*c.rho*Va^2*c.A;

fxAero = -faero*(Drag(alpha,c)+c.CD_delta_e*delta_e);

fyAero = -faero*(c.Cy_beta*beta+c.Cy_delta_r*delta_r);

fzAero = -faero*(Lift(alpha,c)+c.CL_delta_e*delta_e);

TxAero = faero*c.S*c.C_delta_a*delta_a-c.bp*x(4);

TyAero = -faero*c.c*...
    (c.CTy_alpha*alpha+c.CTy_delta_e*delta_e) - c.br * x(5);
TzAero = faero*c.c*...
    (c.CTz_beta*beta+c.CTz_delta_r*delta_r)-c.bq*x(6);
end

%% Drag coefficient
function CD = Drag(alpha,c)
% These values were curve-fit from a drag coefficient graph
CD = c.CD0 + c.CD2 * alpha^2;
end

%% Lift coefficient
function CL = Lift(alpha,c)
% These values were curve-fit from a lift coefficient graph
CL = c.CL0 +  c.CL1/ (c.CL2 + exp(-c.CL3 * (alpha + c.CL4)));
end

%% This function calculates the Runge-Kutta k-slopes for the airplane
function k = AirplaneDynamics(x, fx, fy, fz, Tx, Ty, Tz, c)
% k = AirplaneDynamics(x,fx, fy, fz, Tx, Ty, Tz, c)
%
%AirplaneDynamics calculates the runge kutta k variable for the airplane
% equations of motion give the current state x, the forces, and torques.
%k: the runge kutta k variable (k1, k2, k3, or k4)
%x: the state vector, it has the following 13 state variables:
% x=[u;v;w;p;q;r;xI;yI;zI;e0;e1;e2;e3]
% x(1): u (m/s) is the x-velocity of the airplane in the body frame
% x(2): v (m/s) is the y-velocity of the airplane in the body frame
% x(3): w (m/s) is the z-velocity of the airplane in the body frame
% x(4): p (rad/s) roll rate about the body frame x-axis
% x(5): q (rad/s) yaw rate about the body frame y-axis
% x(6): r (rad/s) pitch rate about the body frame z-axis
% x(7): xI (m) is the north position of the airplane in the inertial frame
% x(8): yI (m) is the east position of the airplane in the inertial frame
% x(9): zI (m) is the down position of the airplane in the inertial frame
% x(10): e0 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(11): e1 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(12): e2 (rad) is a quaternion.  It is related to roll, pitch, and yaw
% x(13): e3 (rad) is a quaternion.  It is related to roll, pitch, and yaw
%fx: (N) force in the x-direction of the body frame
%fy: (N) force in the y-direction of the body frame
%fz: (N) force in the z-direction of the body frame 
%Tx: (N m) Torque about the x-axis in the body frame
%Ty: (N m) Torque about the y-axis in the body frame
%Tz: (N m) Torque about the z-axis in the body frame
%c: a structure of constant parameters with
%c.mass: (kg) mass of the airplane
%c.Jxx: (kg m2) moment of inertia about the body x-axis
%c.Jyy: (kg m2) moment of inertia about the body y-axis
%c.Jzz: (kg m2) moment of inertia about the body z-axis
%c.Jxz: (kg m2) Product of inertia

u=x(1);v=x(2);w=x(3);p=x(4);q=x(5);r=x(6);
e0=x(10);e1=x(11);e2=x(12);e3=x(13);

k13 = [r*v-q*w;p*w-r*u;q*u-p*v]+1/c.mass*[fx;fy;fz];

Jinv = 1/(c.Jxx*c.Jyy*c.Jzz-c.Jyy*c.Jxz^2) * ...
    [c.Jyy*c.Jzz, 0, c.Jxz*c.Jyy;...
    0, c.Jxx*c.Jzz-c.Jxz^2, 0;...
    c.Jyy*c.Jxz, 0, c.Jxx*c.Jyy];
k46 = Jinv * ([q*r*(c.Jyy-c.Jzz)+p*q*c.Jxz+Tx;...
    p*r*(c.Jzz-c.Jxx)-(p^2+q^2)*c.Jxz+Ty;...
    p*q*(c.Jxx-c.Jyy)-q*r*c.Jxz + Tz]);

k79 = [e0^2+e1^2-e2^2-e3^2, 2*(e1*e2-e0*e3), 2*(e0*e2+e1*e3);...
    2*(e0*e3+e1*e2), e0^2-e1^2+e2^2-e3^2, 2*(e2*e3-e0*e1);...
    2*(e1*e3-e0*e2), 2*(e0*e1+e2*e3) e0^2-e1^2-e2^2+e3^2]*[u;v;w];

k1013 = 1/2*[-e1, -e2, -e3;...
    e0, -e3, e2;...
    e3, e0, -e1;...
    -e2, e1, e0] * [p;q;r];

k = [k13;k46;k79;k1013];
end

%% This function calculates the Runge-Kutta k-slopes for the wind gusts
function k = GustDynamics(xGust,wind)
%k = GustDynamics(xGust,Va,wind)
%
%INPUTS:
%xGust: an array with xGust = [uwg; x1; x2; x3; x4; Va] where
%xGust(1): [m/s] uwg is the wind gusts in the body-frame x-axis
%xGust(2): x1 is a state variable for the wind gusts in the y-axis
%xGust(3): x2 is a state variable for the wind gusts in the y-axis
%xGust(4): x3 is a state variable for the wind gusts in the z-axis
%xGust(5): x4 is a state variable for the wind gusts in the z-axis
%xGust(6): [m/s] Va is the previous relative airspeed of the plane
%
%OUTPUT:
%k: an array for Runge-Kutta or Euler solvers, k = f(x,u)

%Intermediate matrices
Auw = -xGust(end)/wind.Lu;
Buw = wind.sigma_u*sqrt(2*xGust(end)/pi/wind.Lu);

Avw = [0,1;-(xGust(end)/wind.Lv)^2,-2*xGust(end)/wind.Lv];
Bvw = [0;wind.sigma_v*sqrt(3*xGust(end)/pi/wind.Lv)];

Aww = [0,1;-(xGust(end)/wind.Lw)^2,-2*xGust(end)/wind.Lw];
Bww = [0;wind.sigma_w*sqrt(3*xGust(end)/pi/wind.Lw)];

%get k
k = [Auw*xGust(1)+Buw*randn;...
    Avw*xGust(2:3)+Bvw*randn;...
    Aww*xGust(4:5)+Bww*randn];
end

%% These functions draw the airplane
function DrawAirplane(xNorth,yEast,zDown,e0,e1,e2, e3)
%get the airplane vertices, faces, and colors
[Vertices, Faces, Colors] = AirplaneGraphics;

% transform vertices from North-East-Down to East-North-Up 
% (for MATLAB rendering)
R_ENU = [0,1,0; ...Convert East to North
    1,0,0; ...Convert North to East
    0,0,-1]; % Convert Down to Up
Vertices = (R_ENU*Vertices')';

%Convert from the body reference frame to the inertial frame
%first rotate, then translate (north is east, east is north, -down is up)
Vertices = BodyToInertialRotation(Vertices, e0, e2, e1, -e3);
Vertices = BodyToInertialTranslation(Vertices, yEast, xNorth, -zDown);

%draw the airplane using the "patch" command
%To learn more about the patch command, type "help patch" in the command
% window
hold on
patch('Vertices', Vertices, 'Faces', Faces,...
     'FaceVertexCData',Colors,'FaceColor','flat');
%north is east, east is north, -down is up
xlabel('East (y)')
ylabel('North (x)')
zlabel('Up (-z)')
view(32,47)  % set the view angle for the figure
axis([yEast-10,yEast+10,xNorth-10,xNorth+10,-zDown-10,-zDown+10]); %set axis limits
pbaspect([1 1 1]) %Set the aspect ratio
grid on
hold off
end

function RotatedPoints = BodyToInertialRotation(points, e0, e1, e2, e3)
%RotatedPoints = BodyToInertialRotation(points, e0, e1, e2, e3)
%
% points: location of points in the body frame that are to be
%  
    %get the quaternion body to inertial frame rotation matrix
    Rb2I = [e0^2+e1^2-e2^2-e3^2, ...
        2*(e1*e2-e0*e3), ...
        2*(e0*e2+e1*e3); ...
        ...
        2*(e0*e3+e1*e2), ...
        e0^2-e1^2+e2^2-e3^2, ...
        2*(e2*e3-e0*e1); ...
        ...
        2*(e1*e3-e0*e2), ...
        2*(e0*e1+e2*e3), ...
        e0^2-e1^2-e2^2+e3^2];
    
    
    %Rotate the points
    RotatedPoints = (Rb2I*points')';
end

function translatedPoints = BodyToInertialTranslation(points, xNorth, yEast, zDown)
translatedPoints = zeros(size(points));
translatedPoints(:,1) = points(:,1)+xNorth;
translatedPoints(:,2) = points(:,2)+yEast;
translatedPoints(:,3) = points(:,3)+zDown;
end

function [V, F, colors] = AirplaneGraphics
% [V, F, colors] = AirplaneGraphics
%
% V is a matrix of vertices (3D location of vertices)
% F is a matrix of faces that use the vertices to define the corners
% colors is a matrix of colors corresponding to the faces
%
%Define the vertices of the airplane, its faces, and its colors


% Define the vertices (physical location of vertices)
% the locations are [xNorth,yUp,zEast]
V = 2.8*[1.261808	-0.010455	-0.189088
1.198821	0.067507	-0.21358
1.194753	0.035838	-0.255876
1.192425	-0.013515	-0.272144
1.192725	-0.0617	-0.25617
1.195541	-0.090313	-0.214056
1.199795	-0.088424	-0.161889
1.203864	-0.056756	-0.119594
1.206193	-0.007403	-0.103326
1.205891	0.040782	-0.119299
1.203076	0.069395	-0.161413
1.107006	-0.006387	-0.072866
1.106604	0.057931	-0.094188
1.102847	0.096124	-0.150402
1.097167	0.093603	-0.220036
1.091736	0.051332	-0.276491
1.088628	-0.014544	-0.298206
1.089029	-0.078862	-0.276885
1.092787	-0.117056	-0.220671
1.098466	-0.114534	-0.151037
1.103897	-0.072264	-0.094581
1.089808	-0.07083	-0.269297
1.092787	-0.117056	-0.220671
1.088628	-0.014544	-0.298206
1.097167	0.093603	-0.220036
1.103897	-0.072264	-0.094581
1.105826	0.049899	-0.101776
1.107006	-0.006387	-0.072866
1.091736	0.051332	-0.276491
1.102847	0.096124	-0.150402
1.098466	-0.114534	-0.151037
-0.073986	1.073582	0.034303
0.010154	1.106043	0.025353
0.026	1.743179	0.040894
0.214974	1.659288	0.032993
0.21047	1.774426	0.038787
0.255844	1.77122	0.044102
0.216989	1.830808	0.052868
0.269575	1.82037	0.055561
0.311468	1.764895	0.057158
0.161404	1.078861	0.019252
0.322114	1.644867	0.057366
0.360781	1.128737	0.058102
0.300492	1.090501	0.029784
0.23807	1.114177	0.018589
0.225535	1.386625	0.025303
0.240708	1.089899	0.093872
0.303019	1.071626	0.087736
0.360781	1.128737	0.058102
0.264241	1.643624	0.079909
0.311468	1.764895	0.057158
0.261172	1.771891	0.064437
0.216028	1.648529	0.082935
0.146607	1.820836	0.058451
0.02589	1.789443	0.055386
-0.007282	1.763344	0.055315
0.019795	1.635702	0.071144
-0.081689	1.795629	0.049923
0.012615	1.106149	0.080341
-0.072209	1.074644	0.065342
0.164659	1.081349	0.090894
0.139456	1.827712	0.050548
-0.081689	1.795629	0.049923
-0.008596	1.762963	0.044225
0.311468	1.764895	0.057158
0.261172	1.771891	0.064437
0.270148	1.820412	0.057989
0.263904	1.819326	0.059359
0.22452	1.823674	0.060951
-0.072209	1.074644	0.065342
-0.043446	1.374796	0.066467
-0.073986	1.073582	0.034303
0.012615	1.106149	0.080341
-0.072209	1.074644	0.065342
0.010154	1.106043	0.025353
0.164659	1.081349	0.090894
0.012615	1.106149	0.080341
0.161404	1.078861	0.019252
0.010154	1.106043	0.025353
0.161404	1.078861	0.019252
0.012615	1.106149	0.080341
0.23807	1.114177	0.018589
0.240708	1.089899	0.093872
0.161404	1.078861	0.019252
0.164659	1.081349	0.090894
0.161404	1.078861	0.019252
0.240708	1.089899	0.093872
0.303019	1.071626	0.087736
0.240708	1.089899	0.093872
0.300492	1.090501	0.029784
0.23807	1.114177	0.018589
0.360781	1.128737	0.058102
0.303019	1.071626	0.087736
0.300492	1.090501	0.029784
-0.073987	1.073582	0.034303
-0.072209	1.074644	0.065342
-0.19867	1.088468	0.049489
-0.081689	1.795629	0.049924
-0.19867	1.088468	0.049489
-0.007282	1.763344	0.055315
-0.072209	1.074644	0.065342
-0.19867	1.088468	0.049489
-0.081689	1.795629	0.049924
-0.073987	1.073582	0.034303
-0.008596	1.762963	0.044225
-0.073987	1.073582	0.034303
-0.008596	1.762963	0.044225
-0.072209	1.074644	0.065342
0.15444	0.813461	0.09639
0.164659	1.081352	0.090894
0.303019	1.071628	0.087736
0.3191	0.823138	0.092971
0.392185	0.708278	0.058704
0.313077	0.690663	0.09676
0.427311	0.511049	0.058787
0.328026	0.488478	0.096901
0.248528	0.537854	0.103612
0.240193	0.683869	0.09862
0.012614	1.10615	0.080341
-0.072209	1.074645	0.065342
0.021979	0.512968	0.086855
-0.084231	0.465183	0.071167
-0.156698	0.456635	0.062202
-0.158059	0.455759	0.035941
-0.087766	0.470796	0.028733
0.019207	0.512774	0.019475
0.133233	0.503347	0.013931
0.161403	1.078861	0.019252
0.23807	1.114178	0.018589
0.300492	1.090502	0.029784
0.392185	0.708278	0.058704
0.427311	0.511049	0.058787
0.345504	0.485655	0.018208
0.245757	0.521732	0.009585
-0.073987	1.073583	0.034303
0.010154	1.106044	0.025353
0.135806	0.492236	0.096172
0.360781	1.128738	0.058102
0.360781	1.128738	0.058102
-0.084231	0.465183	0.071167
-0.156698	0.456635	0.062202
-0.086328	0.463686	0.027921
0.135806	0.492236	0.096172
0.021979	0.512968	0.086855
0.132052	0.489365	0.013416
0.019207	0.512774	0.019475
0.132052	0.489365	0.013416
0.021979	0.512968	0.086855
0.021979	0.512968	0.086855
-0.084231	0.465183	0.071167
0.019207	0.512774	0.019475
-0.086328	0.463686	0.027921
0.019207	0.512774	0.019475
-0.084231	0.465183	0.071167
0.248528	0.537854	0.103612
0.135806	0.492236	0.096172
0.245757	0.521732	0.009585
0.132052	0.489365	0.013416
0.245757	0.521732	0.009585
0.135806	0.492236	0.096172
0.345504	0.485655	0.018208
0.328026	0.488478	0.096901
0.245757	0.521732	0.009585
0.248528	0.537854	0.103612
0.245757	0.521732	0.009585
0.328026	0.488478	0.096901
0.427311	0.511049	0.058787
0.328026	0.488478	0.096901
0.345504	0.485655	0.018208
0.300492	1.090502	0.029784
0.303019	1.071628	0.087736
0.360781	1.128738	0.058102
0.012614	1.10615	0.080341
0.010154	1.106044	0.025353
-0.072209	1.074645	0.065342
0.012614	1.10615	0.080341
0.164659	1.081352	0.090894
0.010154	1.106044	0.025353
0.161403	1.078861	0.019252
0.010154	1.106044	0.025353
0.164659	1.081352	0.090894
0.164659	1.081352	0.090894
0.240708	1.0899	0.093872
0.161403	1.078861	0.019252
0.23807	1.114178	0.018589
0.161403	1.078861	0.019252
0.240708	1.0899	0.093872
0.23807	1.114178	0.018589
0.240708	1.0899	0.093872
0.300492	1.090502	0.029784
0.303019	1.071628	0.087736
0.046564	0.11029	-0.215226
0.039126	0.088365	-0.282798
-0.24523	0.094201	-0.309509
-0.253468	0.11129	-0.244896
-0.353563	0.101751	-0.254635
-0.349086	0.085202	-0.31961
-0.441753	0.065447	-0.328524
-0.466474	0.08552	-0.265515
-0.476756	0.044821	-0.331673
-0.525772	0.048161	-0.289484
-0.50298	-0.01717	-0.334407
-0.547578	-0.015925	-0.298829
-0.528221	-0.079323	-0.289873
-0.479134	-0.078943	-0.33205
-0.469619	-0.116685	-0.266148
-0.444217	-0.101115	-0.329047
-0.358003	-0.130088	-0.255342
-0.35298	-0.118223	-0.320231
-0.249452	-0.126307	-0.310182
-0.258255	-0.138747	-0.245659
0.035194	-0.118031	-0.283429
0.041867	-0.135064	-0.215974
0.04192	-0.125484	-0.215649
0.035274	-0.109929	-0.283109
-0.252184	-0.117679	-0.310093
-0.260451	-0.128998	-0.245578
-0.356522	-0.121029	-0.255342
-0.351896	-0.110244	-0.320226
-0.4398	-0.094821	-0.329122
-0.464052	-0.109037	-0.266242
-0.47356	-0.073977	-0.332154
-0.52069	-0.074216	-0.290019
-0.494581	-0.016915	-0.334588
-0.537411	-0.015669	-0.299049
-0.518435	0.043586	-0.289659
-0.47137	0.040387	-0.331805
-0.461202	0.078382	-0.265654
-0.437579	0.059661	-0.328634
-0.352435	0.0932	-0.254688
-0.34831	0.077732	-0.319652
-0.248296	0.086081	-0.309471
-0.256042	0.102048	-0.244872
0.038894	0.080791	-0.282525
0.046245	0.101235	-0.214957
-0.362717	-0.169832	0.090753
-0.258305	-0.155422	0.034119
-0.222484	-0.187333	0.032504
-0.390193	0.142781	0.139877
-0.355746	0.165392	0.091764
-0.500088	0.134431	0.112607
-0.269295	0.130173	0.155602
-0.243923	0.134878	0.098336
-0.17107	0.103785	0.145802
-0.163042	0.093364	0.096084
-0.118451	0.047702	0.098122
-0.116623	0.057701	0.147925
-0.086559	-0.001658	0.100413
-0.080426	0.000491	0.159741
-0.119021	-0.057649	0.147577
-0.120508	-0.051251	0.097823
-0.174056	-0.107392	0.156938
-0.167002	-0.097147	0.095509
-0.249611	-0.138646	0.097511
-0.285879	-0.134539	0.139533
-0.362717	-0.169832	0.090753
-0.396152	-0.143796	0.139014
-0.505744	-0.137601	0.111787
-0.504295	-0.117868	0.197766
-0.381095	-0.129242	0.173051
-0.385947	-0.120658	0.207031
-0.502093	-0.093374	0.222437
-0.255957	-0.125293	0.157735
-0.081126	-0.101083	0.168568
-0.168649	-0.116319	0.21738
-0.078627	-0.107628	0.215671
-0.173627	-0.083202	0.245722
-0.077287	-0.075783	0.235242
-0.075028	0.003636	0.246652
-0.263363	-0.1191	0.215276
-0.382515	-0.096551	0.238416
-0.262989	-0.090427	0.246326
-0.378371	0.102729	0.239018
-0.378269	0.127639	0.190803
-0.498109	0.098196	0.223015
-0.258313	0.123815	0.216009
-0.259084	0.09738	0.246893
-0.163707	0.121346	0.218097
-0.170021	0.090261	0.246246
-0.074046	0.112682	0.216336
-0.074002	0.082246	0.235718
-0.218041	0.004246	0.257064
-0.268132	-0.048033	0.259196
-0.374218	-0.042665	0.256547
-0.508386	-0.049352	0.236182
-0.623127	-0.086739	0.200257
-0.17206	0.121762	0.180647
-0.649971	0.118029	0.113114
-0.49933	0.120909	0.198486
-0.076889	0.102728	0.169182
-0.607132	-0.089136	-0.267146
-0.613674	-0.016179	-0.289097
-0.534138	-0.068907	-0.294354
-0.604065	0.058395	-0.2667
-0.822026	0.085745	0.116652
-0.619399	0.089769	0.200788
-0.89275	-0.014915	-0.260644
-0.825786	-0.088156	0.11613
-1.017171	-0.003283	0.055757
-1.001313	0.08554	0.050088
-1.024771	0.063136	-0.186214
-1.033442	0.035405	-0.228173
-1.154287	0.075428	0.02518
-1.150263	-0.002228	0.094233
-1.005351	-0.092465	0.049559
-0.655029	-0.12141	0.112393
-0.543224	-0.138983	0.011197
-0.53618	-0.113436	-0.240325
-1.25565	0.068522	-0.097727
-1.165242	0.073912	-0.104409
-1.262025	-0.004143	0.04873
-1.158775	-0.083276	0.030808
-1.01391	-0.100995	-0.020794
-1.028181	-0.087185	-0.186662
-1.1749	-0.092205	-0.091443
-1.503005	0.056102	-0.052201
-1.686288	0.039344	-0.038808
-1.710176	0.031714	-0.012874
-1.704065	0.035447	-0.111263
-1.567087	0.047157	-0.149756
-1.56647	0.046381	-0.107988
-1.495397	0.053508	-0.139922
-1.359874	0.058539	-0.163664
-1.256583	0.066562	-0.165853
-1.169238	0.06399	-0.172539
-1.26764	0.061577	-0.195084
-1.493155	0.051513	-0.203238
-1.612034	0.042822	-0.195424
-1.349637	0.060672	-0.204944
-1.270933	0.018811	-0.225736
-1.173669	0.036103	-0.21298
-1.724733	0.031944	-0.178354
-1.658077	0.016056	-0.257654
-1.731472	0.009732	-0.249765
-1.613699	0.00982	-0.303054
-1.571916	0.012078	-0.266908
-1.458304	0.010513	-0.265246
-1.062896	-0.014351	-0.248548
-1.035663	-0.062511	-0.228464
-1.272498	-0.046911	-0.225929
-1.265581	-0.088882	-0.180924
-1.363076	-0.082624	-0.164084
-1.50593	-0.072868	-0.052585
-1.672494	-0.051359	-0.009827
-1.559264	-0.00611	0.0145
-1.569053	-0.067475	-0.108326
-1.508642	-0.078068	-0.147708
-1.664012	-0.032557	-0.624101
-1.639175	-0.045446	-0.569836
-1.726572	-0.048409	-0.593414
-1.615457	-0.04404	-0.303202
-1.573718	-0.04315	-0.267061
-1.660157	-0.047649	-0.257829
-1.496118	-0.079105	-0.203627
-1.614602	-0.070437	-0.195761
-1.423388	-0.084355	-0.205699
-1.699677	-0.056822	-0.045013
-1.353002	-0.087678	-0.205386
-1.390172	-0.015674	-0.305865
-1.51696	-0.038077	-0.313676
-1.45322	-0.016648	-0.321659
-1.637965	-0.008386	-0.569734
-1.725253	-0.008021	-0.593302
-1.515571	0.004453	-0.313558
-1.76278	-0.038033	-0.596196
-1.711033	-0.026963	-0.008928
-1.726624	-0.058966	-0.178653
-1.73255	-0.041865	-0.249978
-0.29527	-0.137532	-0.249396
0.045929	-0.139258	-0.215199
0.112989	-0.13684	-0.205564
0.115211	-0.141985	-0.096182
0.398231	-0.141074	-0.067464
0.525808	-0.137265	0.01214
0.646327	-0.122177	0.051761
0.84505	-0.116121	-0.003001
0.026695	-0.147394	-0.028885
-0.092549	-0.143361	-0.032558
-0.271232	-0.146689	-0.015682
-0.390395	-0.15034	0.060026
-0.258305	-0.155422	0.034119
0.123719	-0.153899	-0.014436
0.250109	-0.140927	-0.051434
0.9978	-0.049841	0.020544
1.005914	-0.003315	0.028881
0.999722	0.042605	0.020823
0.849688	0.106942	-0.002328
0.651293	0.116619	0.05248
0.531336	0.128644	0.012942
0.403795	0.126484	-0.066656
0.120759	0.12484	-0.095378
0.118158	0.111784	-0.204814
0.051183	0.113392	-0.214438
0.658731	0.085272	0.092969
0.665056	-0.00089	0.104023
0.570186	0.126887	0.040004
0.558989	0.11278	0.08958
0.571256	0.069596	0.110388
0.571194	-0.000571	0.115059
0.655131	-0.087882	0.092448
0.554236	-0.115812	0.088891
0.568331	-0.071093	0.109964
-0.290152	0.108617	-0.248653
-0.46265	0.089531	-0.253831
-0.532059	0.08476	-0.239727
-0.531951	0.036318	-0.294037
-0.550172	-0.016575	-0.301565
-0.537663	0.128479	0.012004
0.032567	0.134963	-0.028033
-0.384282	0.143627	0.060911
-0.252015	0.147063	0.035031
0.129886	0.142678	-0.013541
0.255686	0.127243	-0.050625
0.048823	-0.127266	-0.285046
0.130224	-0.125664	-0.259153
0.233508	-0.129653	-0.249981
0.662402	-0.137009	-0.066536
0.848996	-0.132783	-0.071466
0.995288	-0.092517	-0.017123
0.996775	-0.121313	-0.078268
1.079229	-0.082371	-0.034839
1.08726	-0.090338	-0.089623
1.004971	-0.132669	-0.14341
1.081138	-0.04441	-0.005465
1.094265	-0.111617	-0.16168
0.994719	-0.130386	-0.201664
1.091924	-0.110594	-0.20729
0.995897	-0.096217	-0.276911
1.089517	-0.087688	-0.257673
1.088765	-0.014847	-0.29224
0.989759	-0.052732	-0.307565
0.991308	0.021772	-0.30734
0.999301	0.067493	-0.276418
1.0926	0.060512	-0.257226
0.831433	0.026789	-0.321639
0.648209	0.030693	-0.335396
0.831525	-0.016451	-0.330602
0.829648	-0.059056	-0.321899
0.827289	-0.117496	-0.28095
0.827591	-0.13922	-0.217304
0.643899	-0.127069	-0.294045
0.642896	-0.096978	-0.320523
0.523838	-0.10666	-0.324287
0.646235	-0.064265	-0.335682
0.333294	-0.017477	-0.347291
0.24735	0.036778	-0.335715
0.348771	0.100664	-0.283541
0.340697	0.078636	-0.31362
0.243148	0.073033	-0.300579
0.185366	0.080683	-0.289312
0.196473	0.044903	-0.332369
0.187695	-0.017739	-0.351137
0.245109	-0.071056	-0.33604
0.341839	-0.079075	-0.335499
0.238317	0.10159	-0.249283
0.153037	0.096483	-0.260549
0.667807	0.122933	-0.065753
0.116661	0.097041	-0.256415
0.053473	0.096355	-0.284371
1.096034	0.087069	-0.206693
0.966555	0.106701	-0.200232
1.098486	0.091392	-0.161067
1.010092	0.113618	-0.142668
1.090704	0.075304	-0.089124
1.001522	0.106953	-0.07758
1.082423	0.07128	-0.034376
0.998928	0.082566	-0.016594
1.082797	0.035435	-0.005225
0.854224	0.118666	-0.070707
0.45891	0.06983	-0.323713
0.344418	0.045003	-0.335125
0.648549	0.096517	-0.29337
0.527589	0.073714	-0.323744
0.646253	0.064504	-0.320036
0.832867	0.11453	-0.216539
0.343953	-0.131016	-0.284239
0.336751	-0.111169	-0.314192
0.23945	-0.104788	-0.301115
0.181365	-0.111723	-0.289893
0.193896	-0.079027	-0.332742
-1.319597	0.180846	-0.204517
-1.442541	0.523029	-0.186938
-1.361771	0.533829	-0.195278
-1.425976	0.575174	-0.18015
-1.341238	0.575169	-0.182023
-1.318527	0.535192	-0.188278
-1.275771	0.095915	-0.193958
-1.340926	0.072745	-0.204817
-1.26764	0.061577	-0.195084
-1.414823	0.070737	-0.199719
-1.495717	0.080699	-0.188123
-1.418644	-0.097694	-0.20022
-1.344838	-0.099703	-0.20533
-1.271021	-0.087469	-0.195527
-1.280709	-0.121776	-0.194608
-1.328412	-0.207713	-0.205672
-1.343394	-0.560999	-0.19154
-1.386592	-0.560354	-0.198534
-1.500663	-0.107265	-0.188667
-1.367912	-0.600653	-0.185521
-1.452657	-0.600941	-0.18365
-1.479613	-0.554284	-0.18987
-1.551811	-0.595885	-0.176351
-1.563718	-0.55899	-0.175275
-1.172682	-0.08777	-0.17299
-1.260437	-0.087173	-0.105284
-0.252015	0.147063	0.035031
-0.214868	0.178925	0.033608
0.033018	0.141117	-0.012863
0.259886	0.142665	-0.016132
0.275186	0.181546	0.002829
0.406953	0.17503	0.01691
0.413034	0.13668	-0.011585
0.494394	0.169249	0.032367
0.531336	0.128644	0.012942
0.531106	0.159788	0.058506
0.568378	0.129206	0.053665
-0.222484	-0.187333	0.032504
-0.198534	-0.239842	0.033053
0.267432	-0.191347	0.001704
0.275919	-0.249652	0.005525
0.369163	-0.251699	0.012869
0.39919	-0.183723	0.012073
0.487204	-0.176541	0.031323
0.52435	-0.165125	0.057527
0.467912	-0.309197	0.058329
-0.072472	0.093038	0.172156
-0.134257	0.07946	0.175851
-0.076303	-0.091169	0.171601
-0.137519	-0.077425	0.175379
0.413034	0.13668	-0.011585
0.40713	-0.147276	-0.012441
0.562888	-0.134821	0.052869
0.52435	-0.165125	0.057527
0.525808	-0.137265	0.01214
0.487204	-0.176541	0.031323
0.40713	-0.147276	-0.012441
0.39919	-0.183723	0.012073
0.25372	-0.153852	-0.017027
0.267432	-0.191347	0.001704
0.026913	-0.152452	-0.013749
-0.079632	-0.189845	0.018817
-0.087891	-0.155471	0.003852
-0.222484	-0.187333	0.032504
-0.258305	-0.155422	0.034119
-0.197864	0.172	0.063634
-0.083178	0.170692	0.075896
-0.156699	0.456635	0.062201
-0.084232	0.465183	0.071166
0.021977	0.512968	0.086854
0.135805	0.492237	0.096171
0.248527	0.537853	0.103611
0.212299	0.063228	0.12186
0.416016	0.053413	0.109379
0.414847	-0.000711	0.114846
0.126587	0.051354	0.106473
0.210363	-0.000998	0.111711
0.209643	-0.064502	0.121476
0.413758	-0.055242	0.109051
0.437184	-0.125536	0.101239
0.429465	-0.17189	0.098307
-0.08929	0.104691	0.082104
0.030016	0.048267	0.100239
0.023894	-0.001469	0.10306
0.027943	-0.051411	0.099939
0.124399	-0.053883	0.106155
-0.099415	0.04985	0.093291
-0.086559	-0.001658	0.100413
-0.082629	-0.056181	0.088134
-0.166905	-0.09745	0.094185
-0.205145	-0.178207	0.062579
-0.197251	-0.236954	0.061031
-0.090383	-0.175819	0.074851
0.014801	-0.245398	0.088208
0.124875	-0.237718	0.09776
0.216111	-0.24883	0.107413
0.35285	-0.260404	0.097485
0.394408	-0.27049	0.095749
-0.093741	-0.109362	0.081459
0.469284	-0.000619	0.116103
0.480696	-0.050939	0.115561
0.554236	-0.115812	0.088891
0.562888	-0.134821	0.052869
0.52435	-0.165125	0.057527
0.467912	-0.309197	0.058329
0.328025	0.488477	0.0969
0.43656	0.169342	0.099336
0.441326	0.117392	0.102492
0.482788	0.049694	0.115864
0.571194	-0.000571	0.115059
0.568331	-0.071093	0.109964
0.514182	0.122582	0.085388
0.42731	0.511049	0.058786
0.531106	0.159788	0.058506
0.568378	0.129206	0.053665
0.558989	0.11278	0.08958
0.571256	0.069596	0.110388
0.030016	0.048267	0.100239
-0.080379	0.051998	0.08846
0.023894	-0.001469	0.10306
-0.086559	-0.001658	0.100413
-0.082629	-0.056181	0.088134
0.027943	-0.051411	0.099939
-0.083178	0.170692	0.075896
-0.197864	0.172	0.063634
-0.08929	0.104691	0.082104
-0.243923	0.134878	0.098336
-0.249611	-0.138646	0.097511
-0.093741	-0.109362	0.081459
-0.205145	-0.178207	0.062579
-0.090383	-0.175819	0.074851
0.275186	0.181546	0.002829
0.13205	0.489364	0.013415
0.245756	0.521731	0.009585
0.345503	0.485654	0.018208
0.531106	0.159788	0.058506
0.494394	0.169249	0.032367
0.406953	0.17503	0.01691
0.42731	0.511049	0.058786
0.480659	0.303858	0.060178
0.019206	0.512773	0.019474
-0.188733	0.231528	0.034475
-0.214868	0.178925	0.033608
-0.15806	0.455759	0.03594
-0.086329	0.463686	0.02792
-0.069903	0.102035	0.214774
-0.132897	0.088279	0.197253
-0.07185	0.104752	0.179796
-0.074043	-0.097084	0.214174
-0.135079	-0.070206	0.219169
-0.072832	-0.068303	0.231861
-0.071023	0.003374	0.239318
-0.069863	0.074527	0.232292
-0.132051	0.075409	0.219608
-0.069903	0.102035	0.214774
-0.076156	-0.102332	0.179172
-0.136494	-0.084697	0.196732
-0.074043	-0.097084	0.214174
-1.763794	-0.029064	-0.60805
-1.710219	0.012258	-0.008818
-1.71329	-0.050899	-0.028992
-1.724733	0.031944	-0.178354
-1.73255	-0.041865	-0.249978
-1.731472	0.009732	-0.249765
-1.726624	-0.058966	-0.178653
-0.506214	0.055145	0.236497
-0.372288	0.050155	0.256827
-0.265972	0.055896	0.25951
-0.536443	0.002641	0.229971
-0.623342	0.001903	0.211335
-0.080426	0.000491	0.159741
-0.081288	-0.088732	0.160192
-0.119021	-0.057649	0.147577
-0.077577	0.089768	0.160729
-0.116623	0.057701	0.147925
-0.362717	-0.169832	0.090753
-0.222484	-0.187333	0.032504
-0.249611	-0.138646	0.097511
-1.275865	-0.104622	-0.195067
-1.505475	-0.092613	-0.158938
-1.473685	-0.552103	-0.174722
-1.383879	-0.559685	-0.178371
-1.452657	-0.600941	-0.18365
-1.367912	-0.600653	-0.185521
-1.343394	-0.560999	-0.19154
-1.324361	-0.20644	-0.177411
0.999722	0.042605	0.020823
1.005914	-0.003315	0.028881
1.082797	0.035435	-0.005225
1.081138	-0.04441	-0.005465
0.9978	-0.049841	0.020544
-0.205145	-0.178207	0.062579
-0.222484	-0.187333	0.032504
-0.197251	-0.236954	0.061031
1.094265	-0.111617	-0.16168
1.091924	-0.110594	-0.20729
1.098234	-0.087064	-0.089525
1.088316	-0.036439	-0.292305
1.089213	0.006745	-0.292175
1.096034	0.087069	-0.206693
1.101542	0.072054	-0.089046
1.098486	0.091392	-0.161067
1.088249	-0.015052	-0.297894
1.101542	0.072054	-0.089046
1.098234	-0.087064	-0.089525
-1.525359	0.57015	-0.172882
-1.538939	0.533263	-0.172025
-1.501086	0.545184	-0.171745
-1.439815	0.527751	-0.171712
-1.350146	0.554897	-0.178569
-1.315557	0.181628	-0.176256
-1.271705	0.078746	-0.194521
-1.330817	0.073652	-0.173594
-1.39708	0.07088	-0.165961
-1.499683	0.068198	-0.158506
-1.463218	0.431885	-0.169318
-1.318527	0.535192	-0.188278
-1.525359	0.57015	-0.172882
-1.538939	0.533263	-0.172025
-0.252015	0.147063	0.035031
-0.355746	0.165392	0.091764
-0.214868	0.178925	0.033608
-1.712844	0.037754	-0.04461
-0.355746	0.165392	0.091764
-0.243923	0.134878	0.098336
-0.214868	0.178925	0.033608
1.035859	0.05179	-0.039211
1.034895	-0.034869	-0.017557
1.033944	0.057238	-0.08028
1.034895	-0.034869	-0.017557
1.033466	-0.063304	-0.039559
1.033944	0.057238	-0.08028
1.033466	-0.063304	-0.039559
1.031262	-0.071725	-0.08067
1.033944	0.057238	-0.08028
-0.197864	0.172	0.063634
-0.187528	0.230664	0.062442
-0.214868	0.178925	0.033608
-0.132068	0.087123	0.212148
-0.134564	-0.05795	0.226701
-0.134787	0.069484	0.169345
-0.134564	-0.05795	0.226701
-0.137644	-0.067918	0.168931
-0.134787	0.069484	0.169345
-0.252177	-0.09615	0.030503
-0.345158	-0.081515	-0.210692
-0.248358	0.087525	0.031057
-0.342313	0.055297	-0.210279
0.062885	-0.097052	0.065267
0.066805	0.091474	0.065836
0.091189	-0.093855	-0.146663
0.094658	0.073008	-0.146159
0.098749	-0.095777	-0.212356
0.091189	-0.093855	-0.146663
0.1022	0.070196	-0.211854
0.094658	0.073008	-0.146159
0.1135	-0.017374	-0.339275
0.10769	-0.074985	-0.300563
0.110144	0.043038	-0.300207
0.10769	-0.074985	-0.300563
0.098749	-0.095777	-0.212356
0.110144	0.043038	-0.300207
0.1022	0.070196	-0.211854
1.079973	-0.084351	-0.085983
1.031262	-0.071725	-0.08067
1.082604	-0.074301	-0.036918
1.033466	-0.063304	-0.039559
1.083174	0.069565	-0.085519
1.033944	0.057238	-0.08028
1.079973	-0.084351	-0.085983
1.031262	-0.071725	-0.08067
1.085461	0.063063	-0.036504
1.035859	0.05179	-0.039211
1.083174	0.069565	-0.085519
1.033944	0.057238	-0.08028
1.082604	-0.074301	-0.036918
1.033466	-0.063304	-0.039559
1.084311	-0.040363	-0.010657
1.034895	-0.034869	-0.017557
1.085796	0.031018	-0.010443
1.035999	0.038365	-0.028294
1.085461	0.063063	-0.036504
0.182932	-0.073983	-0.329684
0.10769	-0.074985	-0.300563
0.176521	-0.017792	-0.352315
0.1135	-0.017374	-0.339275
0.176521	-0.017792	-0.352315
0.10769	-0.074985	-0.300563
0.170722	-0.104101	-0.291055
0.10769	-0.074985	-0.300563
0.182932	-0.073983	-0.329684
0.10769	-0.074985	-0.300563
0.170722	-0.104101	-0.291055
0.098749	-0.095777	-0.212356
0.043466	-0.118397	-0.286319
0.091189	-0.093855	-0.146663
0.104253	-0.116988	-0.258308
0.091189	-0.093855	-0.146663
0.043466	-0.118397	-0.286319
0.062885	-0.097052	0.065267
-0.105379	-0.131161	-0.23218
-0.077735	-0.098746	0.049442
0.040925	-0.129232	-0.216477
0.062885	-0.097052	0.065267
0.040925	-0.129232	-0.216477
-0.077735	-0.098746	0.049442
-0.286854	-0.127764	-0.250964
-0.252177	-0.09615	0.030503
-0.105379	-0.131161	-0.23218
-0.077735	-0.098746	0.049442
-0.105379	-0.131161	-0.23218
-0.252177	-0.09615	0.030503
-0.345158	-0.081515	-0.210692
-0.252177	-0.09615	0.030503
-0.286854	-0.127764	-0.250964
-0.449922	0.079019	-0.267066
-0.342313	0.055297	-0.210279
-0.504657	0.045356	-0.291234
-0.524219	0.018785	-0.300467
-0.342313	0.055297	-0.210279
-0.531917	-0.016625	-0.303381
-0.531917	-0.016625	-0.303381
-0.342313	0.055297	-0.210279
-0.525687	-0.051821	-0.30068
-0.525687	-0.051821	-0.30068
-0.342313	0.055297	-0.210279
-0.507216	-0.077697	-0.291605
-0.342313	0.055297	-0.210279
-0.345158	-0.081515	-0.210692
-0.507216	-0.077697	-0.291605
-0.507216	-0.077697	-0.291605
-0.345158	-0.081515	-0.210692
-0.453842	-0.10953	-0.267634
-0.282144	0.098747	-0.25028
-0.342313	0.055297	-0.210279
-0.449922	0.079019	-0.267066
-0.100493	0.103809	-0.231471
-0.073773	0.091787	0.050017
-0.282144	0.098747	-0.25028
-0.248358	0.087525	0.031057
-0.282144	0.098747	-0.25028
-0.073773	0.091787	0.050017
0.04576	0.103263	-0.215774
0.066805	0.091474	0.065836
-0.100493	0.103809	-0.231471
-0.073773	0.091787	0.050017
-0.100493	0.103809	-0.231471
0.066805	0.091474	0.065836
0.047745	0.087382	-0.285698
0.094658	0.073008	-0.146159
0.04576	0.103263	-0.215774
0.066805	0.091474	0.065836
0.04576	0.103263	-0.215774
0.094658	0.073008	-0.146159
0.108518	0.088099	-0.257689
0.1022	0.070196	-0.211854
0.047745	0.087382	-0.285698
0.174404	0.072955	-0.290521
0.110144	0.043038	-0.300207
0.143437	0.087576	-0.261791
0.1022	0.070196	-0.211854
0.143437	0.087576	-0.261791
0.110144	0.043038	-0.300207
0.185303	0.04006	-0.329341
0.110144	0.043038	-0.300207
0.174404	0.072955	-0.290521
0.176521	-0.017792	-0.352315
0.1135	-0.017374	-0.339275
0.185303	0.04006	-0.329341
0.110144	0.043038	-0.300207
0.185303	0.04006	-0.329341
0.1135	-0.017374	-0.339275
0.066805	0.091474	0.065836
0.062885	-0.097052	0.065267
-0.073773	0.091787	0.050017
-0.077735	-0.098746	0.049442
-0.077735	-0.098746	0.049442
-0.252177	-0.09615	0.030503
-0.073773	0.091787	0.050017
-0.248358	0.087525	0.031057
-0.213986	0.083969	-0.086834
-0.217797	-0.099248	-0.087387
-0.296908	0.044444	-0.323182
-0.299432	-0.076936	-0.323549
-0.083733	0.079402	-0.088116
-0.087349	-0.094552	-0.088641
-0.213986	0.083969	-0.086834
-0.217797	-0.099248	-0.087387
-0.071201	0.081725	0.027821
-0.074741	-0.088475	0.027308
-0.083733	0.079402	-0.088116
-0.087349	-0.094552	-0.088641
-0.310609	0.044647	-0.317247
-0.213986	0.083969	-0.086834
-0.264574	0.072691	0.030758
-0.26774	-0.079555	0.0303
-0.313133	-0.076733	-0.317613
-0.217797	-0.099248	-0.087387
-0.26774	-0.079555	0.0303
-0.264574	0.072691	0.030758
-0.326835	-0.07653	-0.311678
-0.324311	0.04485	-0.311312
-0.248358	0.087525	0.031057
-0.342313	0.055297	-0.210279
-0.282144	0.098747	-0.25028
-0.453842	-0.10953	-0.267634
-0.345158	-0.081515	-0.210692
-0.286854	-0.127764	-0.250964
0.50891	-0.126	0.08465
-0.091537	-0.247101	0.026496
-0.091742	-0.243949	0.072794
-0.198534	-0.239842	0.033053
0.013365	-0.249734	0.014526
0.014801	-0.245398	0.088208
-0.091537	-0.247101	0.026496
-0.091742	-0.243949	0.072794
-0.091537	-0.247101	0.026496
0.014801	-0.245398	0.088208
0.124875	-0.237718	0.09776
0.014801	-0.245398	0.088208
0.118389	-0.247089	0.008995
0.013365	-0.249734	0.014526
0.118389	-0.247089	0.008995
0.014801	-0.245398	0.088208
0.275919	-0.249652	0.005525
0.216111	-0.24883	0.107413
0.118389	-0.247089	0.008995
0.124875	-0.237718	0.09776
0.118389	-0.247089	0.008995
0.216111	-0.24883	0.107413
0.369163	-0.251699	0.012869
0.35285	-0.260404	0.097485
0.275919	-0.249652	0.005525
0.216111	-0.24883	0.107413
0.275919	-0.249652	0.005525
0.35285	-0.260404	0.097485
0.467912	-0.309197	0.058329
0.394408	-0.27049	0.095749
0.369163	-0.251699	0.012869
0.394408	-0.27049	0.095749
0.35285	-0.260404	0.097485
0.369163	-0.251699	0.012869
0.345503	0.485654	0.018208
0.328025	0.488477	0.0969
0.42731	0.511049	0.058786
-0.086329	0.463686	0.02792
-0.15806	0.455759	0.03594
-0.084232	0.465183	0.071166
-0.084232	0.465183	0.071166
0.021977	0.512968	0.086854
-0.086329	0.463686	0.02792
0.019206	0.512773	0.019474
-0.086329	0.463686	0.02792
0.021977	0.512968	0.086854
0.021977	0.512968	0.086854
0.135805	0.492237	0.096171
0.019206	0.512773	0.019474
0.13205	0.489364	0.013415
0.019206	0.512773	0.019474
0.135805	0.492237	0.096171
0.13205	0.489364	0.013415
0.135805	0.492237	0.096171
0.245756	0.521731	0.009585
0.248527	0.537853	0.103611
0.245756	0.521731	0.009585
0.135805	0.492237	0.096171
0.248527	0.537853	0.103611
0.328025	0.488477	0.0969
0.245756	0.521731	0.009585
0.345503	0.485654	0.018208
0.245756	0.521731	0.009585
0.328025	0.488477	0.0969
-1.33836	-0.463547	-0.197142
0.500133	0.218203	0.053349
-1.563718	-0.55899	-0.175275
-1.551811	-0.595885	-0.176351
-1.530031	-0.556637	-0.174206
-0.193966	1.067469	0.054202
-0.072981	1.053731	0.044193
-0.34234	0.179866	0.048337
-0.196569	0.16166	0.040471
-0.072073	1.054443	0.066076
-0.193966	1.067469	0.054202
-0.196089	0.16231	0.058462
-0.34234	0.179866	0.048337
-0.145397	-0.797678	0.033334
-0.118807	-1.081927	0.027802
-0.117026	-1.080743	0.058842
0.467913	-0.309197	0.058329
0.369164	-0.2517	0.012869
0.27592	-0.249652	0.005526
0.362613	-0.713974	0.054415
0.2546	-1.116552	0.023128
0.313718	-1.134639	0.051276
-0.118807	-1.081927	0.027802
-0.035033	-1.085751	0.020569
0.116785	-1.067016	0.01278
0.192541	-1.075415	0.011985
0.216111	-0.248831	0.107413
0.352851	-0.260404	0.097485
-0.117026	-1.080743	0.058842
-0.033974	-1.083914	0.070709
0.120044	-1.06427	0.084424
0.284291	-0.693739	0.092585
0.307651	-0.491468	0.093945
0.394409	-0.27049	0.09575
0.467913	-0.309197	0.058329
0.362613	-0.713974	0.054415
0.257613	-1.112074	0.081149
0.313718	-1.134639	0.051276
0.195746	-1.072466	0.08735
0.124875	-0.237718	0.097761
-0.19725	-0.236954	0.061032
0.014802	-0.245398	0.088208
-0.197892	-0.238398	0.047043
-0.145397	-0.797678	0.033334
-0.117026	-1.080743	0.058842
-0.198534	-0.239842	0.033054
-0.033108	-1.092795	0.073708
-0.117026	-1.080743	0.058842
-0.035611	-1.094914	0.018715
0.195746	-1.072466	0.08735
0.120044	-1.06427	0.084424
0.192541	-1.075415	0.011985
0.116785	-1.067016	0.01278
0.116785	-1.067016	0.01278
0.120044	-1.06427	0.084424
-0.035611	-1.094914	0.018715
-0.033108	-1.092795	0.073708
0.257613	-1.112074	0.081149
0.195746	-1.072466	0.08735
0.2546	-1.116552	0.023128
0.192541	-1.075415	0.011985
0.2546	-1.116552	0.023128
0.195746	-1.072466	0.08735
0.313718	-1.134639	0.051276
0.257613	-1.112074	0.081149
0.2546	-1.116552	0.023128
-0.091741	-0.24395	0.072795
0.014802	-0.245398	0.088208
-0.091537	-0.247102	0.026497
0.013365	-0.249735	0.014527
-0.091537	-0.247102	0.026497
0.014802	-0.245398	0.088208
-0.091741	-0.24395	0.072795
-0.091537	-0.247102	0.026497
-0.19725	-0.236954	0.061032
0.27592	-0.249652	0.005526
0.11839	-0.247089	0.008996
0.216111	-0.248831	0.107413
0.124875	-0.237718	0.097761
0.216111	-0.248831	0.107413
0.11839	-0.247089	0.008996
0.124875	-0.237718	0.097761
0.11839	-0.247089	0.008996
0.014802	-0.245398	0.088208
0.013365	-0.249735	0.014527
0.014802	-0.245398	0.088208
0.11839	-0.247089	0.008996
0.467913	-0.309197	0.058329
0.369164	-0.2517	0.012869
0.394409	-0.27049	0.09575
0.352851	-0.260404	0.097485
0.369164	-0.2517	0.012869
0.216111	-0.248831	0.107413
0.27592	-0.249652	0.005526
0.216111	-0.248831	0.107413
0.369164	-0.2517	0.012869
0.394409	-0.27049	0.09575
0.369164	-0.2517	0.012869
0.352851	-0.260404	0.097485
0.136517	-1.782141	0.028061
0.182034	-1.778474	0.033396
0.140712	-1.83751	0.041805
0.193739	-1.826784	0.044561
0.237942	-1.771111	0.046494
0.253582	-1.651017	0.047426
0.313718	-1.134638	0.051276
0.2546	-1.116551	0.023128
0.192541	-1.075415	0.011986
0.116784	-1.067016	0.01278
-0.089549	-1.771347	0.033759
-0.156516	-1.803041	0.03907
0.063302	-1.834712	0.039503
0.071303	-1.645591	0.072676
0.195746	-1.072465	0.08735
0.257613	-1.112074	0.081149
0.313718	-1.134638	0.051276
0.195792	-1.648244	0.069981
0.237942	-1.771111	0.046494
0.187365	-1.777666	0.053733
0.147379	-1.653013	0.072977
0.070751	-1.82725	0.047448
-0.057443	-1.806943	0.043733
-0.08843	-1.771046	0.044821
-0.048347	-1.641368	0.06126
-0.156516	-1.803041	0.03907
-0.033108	-1.092794	0.073708
-0.117027	-1.080743	0.058843
0.120044	-1.06427	0.084424
0.194313	-1.82665	0.04699
0.491928	-0.221456	0.052002
0.187365	-1.777666	0.053733
0.237942	-1.771111	0.046494
-0.035611	-1.094914	0.018715
-0.118807	-1.081927	0.027802
0.148552	-1.829776	0.049932
0.188115	-1.825475	0.048367
-0.117027	-1.080743	0.058843
-0.118807	-1.081927	0.027802
-0.100328	-1.380841	0.058147
0.116784	-1.067016	0.01278
-0.118807	-1.081927	0.027802
0.2546	-1.116551	0.023128
-0.118807	-1.081927	0.027802
-0.033108	-1.092794	0.073708
0.2546	-1.116551	0.023128
-0.033108	-1.092794	0.073708
0.195746	-1.072465	0.08735
0.2546	-1.116551	0.023128
0.195746	-1.072465	0.08735
0.257613	-1.112074	0.081149
0.2546	-1.116551	0.023128
0.2546	-1.116551	0.023128
0.257613	-1.112074	0.081149
0.313718	-1.134638	0.051276
-0.118806	-1.081927	0.027802
-0.244091	-1.095932	0.0429
-0.117026	-1.080743	0.058843
-0.156516	-1.803041	0.03907
-0.244091	-1.095932	0.0429
-0.089548	-1.771347	0.033759
-0.118806	-1.081927	0.027802
-0.244091	-1.095932	0.0429
-0.156516	-1.803041	0.03907
-0.117026	-1.080743	0.058843
-0.088429	-1.771046	0.044821
-1.391871	-0.094669	-0.166784
-0.117026	-1.080743	0.058843
-0.088429	-1.771046	0.044821
-0.118806	-1.081927	0.027802
-1.419819	0.057023	-0.205249
-1.45969	-0.040683	-0.265365
-0.855885	-0.089021	0.100352
0.4553	-0.102892	-0.32423
-0.465377	-0.121942	-0.243397
0.564799	-0.133483	0.039228
-0.204599	-0.172377	0.039487
-0.123902	-1.064854	0.037952
-0.351216	-0.189321	0.04725
-0.245515	-1.07729	0.047884
-1.336953	-0.463748	-0.186088
-0.351216	-0.189321	0.04725
-0.245515	-1.07729	0.047884
-0.204118	-0.17172	0.057478
-0.12299	-1.063976	0.059836
-0.079363	0.160383	0.011559
-1.316384	0.437251	-0.183418
-1.49861	0.080259	-0.165754
-1.634954	0.070834	-0.169095
-1.463217	0.431884	-0.169318
-1.538939	0.533262	-0.172026
-1.504094	0.544286	-0.170951
-1.439814	0.52775	-0.171712
-1.44254	0.523028	-0.186938
-1.538939	0.533262	-0.172026
-1.482589	0.178745	-0.18994
-1.614825	0.167139	-0.168413
-1.614825	0.167139	-0.168413
-1.482589	0.178745	-0.18994
-1.634954	0.070834	-0.169095
-1.495717	0.080698	-0.188123
-0.119239	-0.062296	0.094523
-1.638756	-0.096676	-0.169593
-1.507108	-0.105202	-0.166213
-1.563718	-0.55899	-0.175275
-1.473686	-0.552102	-0.174722
-1.479614	-0.554284	-0.18987
-1.563718	-0.55899	-0.175275
-1.496048	-0.205399	-0.190981
-1.622993	-0.192863	-0.169484
-1.496048	-0.205399	-0.190981
-1.622993	-0.192863	-0.169484
-1.500664	-0.107265	-0.188668
-1.638756	-0.096676	-0.169593
0.833684	0.077155	-0.288473
-1.176193	-0.058023	-0.220115
-1.724732	0.031945	-0.178353
-1.826084	0.017717	-0.096926
-1.86504	0.010797	-0.160168
-1.731472	0.009732	-0.249765
-1.906235	-0.016376	-0.231164
-1.896353	-0.005179	-0.272826
-1.875353	-0.027493	-0.546353
-1.897293	-0.03046	-0.272892
-1.763561	-0.03477	-0.603356
-1.807567	-0.031039	-0.610297
-1.856197	-0.029206	-0.597007
-1.757229	-0.020145	-0.557745
-1.705833	-0.058134	-0.108429
-1.911294	-0.013224	-0.157963
-1.317974	0.437853	-0.194455
-1.827421	-0.03943	-0.097096
-1.714774	-0.055068	-0.044888
-1.711804	-0.046729	-0.013095
-1.818273	-0.008374	-0.028587
-1.710261	-0.007197	-0.004761
-1.710175	0.031714	-0.012874
-1.712844	0.037754	-0.04461
-1.867209	-0.009253	-0.050233
-1.897838	-0.010861	-0.093126
-1.726623	-0.058966	-0.178653
-1.732549	-0.041865	-0.249978
-1.323664	-0.098894	-0.177881
-1.731472	0.009732	-0.249765
-1.732549	-0.041865	-0.249978
-1.724732	0.031945	-0.178353
-1.71151	0.034734	-0.028742
-1.711032	-0.026963	-0.008928
-1.714774	-0.055068	-0.044888
-1.726623	-0.058966	-0.178653
-1.763794	-0.029064	-0.60805];
    
    %define the faces
    F = [31	29	23
464	462	463
457	475	390
289	241	288
593	587	594
50	47	48
974	973	972
735	734	736
1009	1008	1006
309	324	323
680	682	679
36	37	35
19	18	5
230	231	232
985	980	979
456	458	392
470	448	447
443	476	1125
287	244	242
1136	1133	1134
283	282	648
315	314	308
553	588	587
746	745	747
259	258	257
90	89	91
964	962	963
230	232	233
47	53	61
542	544	543
662	661	663
1072	1069	1067
471	446	448
342	341	506
100	98	99
30	25	26
369	370	378
412	413	391
265	270	252
413	531	390
553	555	554
39	37	38
1170	1172	1181
682	681	679
328	319	320
984	985	979
495	953	496
983	991	979
68	69	52
190	188	189
70	71	72
73	74	75
76	77	78
79	80	81
82	83	84
85	86	87
92	93	94
95	96	97
854	856	855
871	869	870
510	509	511
106	107	108
750	748	749
1178	1175	1174
445	452	446
116	115	114
501	502	498
262	271	285
248	250	251
14	2	15
438	431	428
1172	1174	1175
440	426	423
1164	1162	1163
1192	1172	1193
364	362	335
585	561	578
690	1138	697
62	36	34
303	318	345
622	624	625
647	284	283
303	345	311
409	391	393
4	1	5
126	135	128
372	383	382
117	114	118
267	265	266
343	313	312
1176	1173	1171
648	279	277
140	141	142
143	144	145
146	147	148
149	150	151
152	153	154
155	156	157
158	159	160
161	162	163
164	165	166
167	168	169
170	171	172
173	174	175
176	177	178
179	180	181
182	183	184
185	186	187
234	235	233
26	27	30
273	647	648
248	247	249
566	557	556
318	366	345
441	443	442
612	623	622
351	350	349
120	121	109
505	314	315
522	520	523
196	194	197
135	136	128
573	571	572
408	309	316
417	374	373
32	64	34
197	198	196
199	196	198
198	200	199
201	199	200
200	202	201
203	201	202
203	202	204
205	204	202
204	205	206
207	206	205
206	207	208
209	208	207
1201	1196	1195
693	691	694
334	333	1179
445	454	453
477	454	1125
276	278	287
219	220	218
221	218	220
220	222	221
223	221	222
222	224	223
225	223	224
350	348	349
1118	660	666
225	224	226
227	226	224
226	227	228
229	228	227
228	229	230
231	230	229
422	421	420
42	44	46
134	129	130
556	553	551
344	357	343
286	298	259
452	445	453
555	558	559
1067	1066	1065
1178	1174	1173
388	387	457
572	574	573
271	284	285
330	338	359
1202	1195	1196
388	397	394
300	288	408
519	518	520
481	488	490
242	239	274
863	861	862
1085	1061	1052
413	390	392
499	953	502
236	237	238
239	240	241
239	242	240
281	279	269
403	405	310
245	243	244
245	244	246
247	246	244
263	261	260
482	484	483
1181	1192	1183
250	252	251
253	251	252
253	252	254
643	645	644
257	256	255
256	257	258
259	257	260
257	255	260
259	261	262
372	382	377
250	264	252
291	339	297
288	295	289
268	267	266
12	10	13
324	326	323
321	319	317
642	644	645
45	41	46
1176	1193	1175
281	280	279
379	307	369
100	99	101
628	626	627
262	286	259
528	529	527
758	757	759
239	289	274
322	316	323
409	408	411
290	247	244
252	270	263
1074	1073	1065
1071	1072	1065
291	292	293
1191	1169	1170
306	1124	307
436	470	473
542	543	541
365	1179	363
746	744	745
535	534	536
535	533	534
446	471	445
275	289	296
289	275	274
1065	1076	1074
125	135	126
300	299	295
373	532	383
1164	1163	1165
272	271	270
304	300	303
300	304	299
304	305	299
312	345	344
1198	1200	1197
309	323	316
465	464	463
1126	308	293
291	293	308
471	448	470
304	303	311
304	311	312
304	312	305
436	437	444
515	516	517
1053	1057	1056
1062	1086	1085
1069	1072	1071
678	677	675
871	870	872
8	7	1
1186	1188	1189
513	514	515
499	492	496
1160	1159	1161
427	428	429
353	355	367
487	481	1182
648	647	283
761	762	760
377	378	370
379	378	381
360	352	351
1195	1197	1201
1068	1069	1070
612	614	615
387	388	394
324	309	325
272	267	283
976	998	977
283	284	272
728	727	725
1052	1062	1085
696	695	691
330	1122	329
326	330	329
326	324	331
352	353	351
367	355	1180
215	216	217
287	290	244
193	195	192
1167	338	339
134	130	133
57	56	60
327	337	336
336	333	327
328	327	333
313	305	312
10	1	11
1167	341	340
20	19	6
675	676	678
31	26	25
660	661	1132
21	7	8
370	371	372
403	409	393
34	35	46
310	408	403
352	354	353
355	353	354
954	618	615
558	576	577
537	535	536
1194	666	659
380	307	379
443	1125	444
341	358	340
340	356	354
354	1123	340
20	6	7
1123	361	359
522	521	520
1182	481	491
1178	1176	1175
360	349	361
437	436	435
1076	1065	1078
475	472	447
364	361	362
294	297	302
337	361	364
1112	1111	1113
1061	1060	1052
289	239	241
1178	1176	1177
336	337	364
365	363	348
425	422	423
33	32	34
350	365	348
273	276	274
661	957	663
21	20	7
112	114	113
505	1167	339
587	586	553
450	456	455
985	986	980
34	36	35
686	687	688
2	1	3
312	344	343
552	553	586
336	364	335
455	475	447
384	400	376
1054	1064	1052
474	473	472
590	597	589
360	1123	352
353	365	351
592	595	594
259	306	258
299	298	650
212	213	211
566	565	575
549	548	547
381	256	380
380	256	258
327	328	320
15	3	16
1184	1190	1183
34	46	33
348	361	349
471	470	445
1153	569	568
1141	1143	1142
1155	1156	1157
378	379	369
248	246	247
396	397	388
398	394	397
398	399	394
395	394	399
395	399	400
395	384	385
1127	375	401
400	402	401
402	400	399
1192	1184	1183
984	989	988
405	404	406
294	405	406
294	406	292
407	292	406
407	293	292
350	351	365
421	424	419
348	362	361
860	857	858
501	503	502
661	662	1132
273	274	275
1136	1135	1133
35	37	42
473	474	436
1152	1151	1149
395	400	384
117	116	114
646	649	285
240	410	241
240	411	410
313	307	305
393	392	458
666	1194	1118
41	33	46
381	380	379
1168	1170	1169
1198	1199	1200
513	512	514
418	376	417
376	418	419
420	419	418
419	420	421
438	437	431
52	50	51
332	328	333
357	1180	346
1169	1190	1189
52	69	54
961	958	959
430	429	428
280	290	278
479	416	415
430	433	434
122	121	120
320	319	321
589	580	590
325	301	302
1082	522	523
128	127	126
17	4	5
625	613	622
548	546	547
540	542	541
258	306	307
370	415	371
994	982	983
584	583	889
448	449	447
477	453	454
451	450	446
114	115	113
580	581	590
613	614	612
9	1	10
455	447	449
137	117	118
1138	690	689
612	615	618
761	758	759
242	243	240
393	458	459
273	275	646
331	338	330
55	58	56
434	433	460
460	461	462
463	462	461
49	51	50
467	466	465
970	969	972
467	387	386
387	467	469
469	457	387
493	496	492
271	261	270
57	55	56
465	469	467
267	272	265
359	340	1123
375	417	376
971	978	977
307	380	258
1052	1053	1054
330	359	337
444	1125	445
296	289	295
631	629	630
36	62	38
193	194	195
218	217	216
998	971	977
1117	1116	1114
285	284	646
416	476	440
461	475	463
686	689	690
439	428	440
732	731	729
288	241	408
1062	1064	1063
386	395	385
1142	1140	1141
17	5	18
665	662	664
298	306	259
338	302	297
477	476	478
478	453	477
478	479	453
480	453	479
254	263	255
1062	1052	1064
528	530	529
703	641	640
242	244	243
132	133	131
485	483	484
485	486	483
486	1182	483
476	441	440
488	481	487
487	489	488
46	35	42
547	562	563
43	44	42
436	444	445
423	418	440
1127	417	375
391	413	392
495	493	494
493	495	496
435	433	432
9	8	1
1181	1191	1170
497	498	953
497	500	498
500	501	498
504	502	503
373	440	417
255	263	260
314	505	339
1189	1168	1169
386	394	395
7	6	1
25	29	31
650	295	299
665	1132	662
105	104	102
269	282	267
234	233	232
507	508	1137
663	664	662
430	428	431
576	558	575
511	512	510
513	510	512
353	367	368
1076	1075	1074
750	749	751
442	443	444
105	102	103
558	566	575
524	1082	523
524	525	1082
1083	1081	1084
755	752	753
585	584	561
531	389	390
1075	1077	1074
446	449	448
301	310	405
532	373	374
66	65	67
1058	1057	1059
277	278	276
1057	1060	1059
537	538	539
540	539	538
574	575	565
335	1179	333
992	994	983
545	543	544
1179	335	363
248	249	250
416	478	476
270	261	263
383	372	371
341	505	506
867	865	866
335	362	363
593	586	587
371	440	373
13	11	14
285	286	262
418	417	440
412	391	409
437	435	432
346	354	347
1065	1073	1071
328	332	319
475	457	469
692	691	693
1055	1054	1053
438	442	444
1076	1078	1079
634	635	633
964	963	965
208	209	210
553	556	557
557	555	553
557	558	555
57	61	53
637	638	636
650	298	286
561	889	560
556	564	566
422	420	423
461	433	1166
978	971	970
418	423	420
558	557	566
255	256	254
269	279	282
1078	1065	1080
995	996	997
1004	1002	1003
468	466	467
1065	1072	1067
531	413	412
463	475	469
435	1166	433
279	278	277
472	475	1166
14	11	2
63	62	64
555	580	554
555	559	580
581	580	559
581	559	560
581	889	582
582	889	583
644	639	643
577	578	561
343	347	342
703	642	641
1141	1144	1143
277	273	648
589	588	554
589	554	580
515	514	516
565	579	573
591	590	581
581	582	591
592	587	588
1065	1066	1080
277	276	273
1130	1128	1129
355	354	1180
596	595	592
589	592	588
596	589	597
598	599	600
601	600	599
601	602	600
603	600	602
604	605	606
607	606	605
607	599	606
602	608	609
608	610	609
611	609	610
376	400	375
1172	1192	1181
954	615	620
659	666	1132
463	469	465
761	760	758
366	357	344
617	954	616
617	618	954
42	37	40
619	620	615
647	273	646
430	432	433
480	452	453
565	569	579
631	630	632
632	630	633
630	634	633
19	5	6
15	2	3
359	338	340
279	648	282
438	444	437
1186	1189	1190
474	435	436
212	211	210
291	297	292
295	288	300
285	649	286
650	286	649
450	455	449
400	401	375
452	451	446
646	296	649
457	396	388
650	649	296
651	652	653
654	651	655
656	657	658
130	138	131
1068	1067	1069
1126	313	315
475	392	390
732	729	730
37	36	38
1057	1052	1060
242	274	276
316	318	303
1053	1052	1057
482	1182	491
445	1125	454
667	668	669
11	1	2
671	670	668
672	673	674
683	684	685
339	291	314
384	376	419
50	53	47
64	62	34
119	120	109
1186	1190	1184
343	315	313
1146	1145	1147
294	302	301
699	698	482
700	701	702
704	705	706
707	708	709
710	711	712
713	714	715
716	717	718
719	720	721
722	723	724
1146	1147	1148
737	738	739
562	567	563
758	756	757
57	59	61
349	360	351
553	552	551
1188	1186	1187
763	764	765
766	767	768
769	770	771
772	773	774
775	776	777
778	779	780
781	782	783
784	785	786
787	788	789
790	791	792
793	794	795
796	797	798
799	800	801
802	803	804
805	806	807
808	809	810
811	812	813
814	815	816
817	818	819
820	821	822
823	824	825
826	827	828
829	830	831
832	833	834
835	836	837
838	839	840
841	842	843
844	845	846
847	848	849
850	851	852
441	439	440
12	9	10
1155	1154	1156
875	874	873
876	877	878
883	884	885
886	887	888
625	621	613
308	1126	315
558	560	559
511	509	1137
741	742	740
321	317	316
377	370	372
551	550	547
331	325	302
735	733	734
331	330	326
1173	1174	1172
890	891	892
893	894	895
896	897	898
899	900	901
902	903	904
905	906	907
908	909	910
911	912	913
914	915	916
917	918	919
920	921	922
923	924	925
926	927	928
929	930	931
932	933	934
935	936	937
938	939	940
941	942	943
944	945	946
947	948	949
950	951	952
563	564	556
343	357	346
955	956	957
956	663	957
863	862	864
1160	1158	1159
362	348	363
592	594	587
966	967	968
408	410	411
973	970	972
45	46	44
427	426	428
338	331	302
208	210	211
645	641	642
267	268	269
755	754	752
305	1124	299
430	437	432
1171	1172	1170
12	8	9
475	461	1166
284	271	272
290	287	278
264	266	265
479	478	416
52	53	50
558	577	561
1186	1184	1185
470	436	445
121	137	118
218	216	219
490	491	481
986	985	987
307	313	369
333	336	335
29	22	23
307	1124	305
472	1166	474
1112	1110	1111
241	410	408
691	1138	696
301	405	294
394	386	387
570	569	1153
113	139	112
666	660	1132
1117	1114	1115
547	556	551
344	345	366
415	416	371
999	1000	1001
1010	1011	1012
1013	1014	1015
1016	1017	1018
1019	1020	1021
1022	1023	1024
1025	1026	1027
1028	1029	1030
1031	1032	1033
1034	1035	1036
1037	1038	1039
1040	1041	1042
1043	1044	1045
1046	1047	1048
1049	1050	1051
668	670	669
1140	1139	1141
133	130	131
540	541	539
90	88	89
689	696	1138
129	134	128
294	292	297
6	5	1
373	383	371
482	698	484
301	325	310
428	439	442
691	695	694
13	10	11
320	321	322
1124	306	298
377	381	378
532	382	383
1200	1201	1197
370	414	415
537	536	538
271	262	261
584	889	561
961	960	958
1183	1191	1181
1009	1006	1007
476	477	1125
322	327	320
392	475	455
291	308	314
439	441	442
339	338	297
461	460	433
1173	1172	1171
1138	691	692
408	303	300
519	520	521
505	341	1167
109	110	119
953	498	502
496	953	499
416	440	371
455	456	392
424	384	419
1186	1185	1187
1088	1071	1087
1089	1090	1091
1092	1093	1094
1095	1096	1097
1098	1099	1100
1101	1102	1103
1104	1105	1106
1107	1108	1109
446	450	449
1119	1120	1121
741	743	742
428	426	440
650	296	295
728	725	726
425	426	427
613	612	622
272	270	265
332	333	334
52	54	53
854	855	853
118	109	121
275	296	646
991	989	984
49	50	48
1076	1079	1075
284	647	646
573	574	565
1172	1175	1193
1152	1149	1150
392	393	391
867	866	868
989	990	988
324	325	331
139	111	112
465	466	464
3	1	4
252	264	265
425	423	426
438	428	442
550	549	547
1168	1171	1170
282	283	267
564	565	566
1069	1071	1070
408	409	403
447	473	470
994	993	981
431	437	430
123	122	120
215	217	214
196	195	194
1082	526	522
279	280	278
16	3	4
1053	1056	1055
365	353	368
53	54	57
483	1182	482
37	39	40
325	309	310
506	505	315
190	189	191
16	4	17
882	881	879
978	970	973
563	556	547
506	343	342
979	991	984
57	60	59
1191	1183	1190
979	992	983
261	259	260
1073	1087	1071
390	389	457
1004	1003	1005
1130	1129	1131
1191	1190	1169
312	311	345
12	21	8
860	859	857
468	467	386
287	242	276
882	879	880
443	441	476
553	554	588
26	28	27
408	310	309
403	404	405
322	321	316
125	124	135
976	975	998
330	327	1122
1167	340	338
327	330	337
340	358	356
354	352	1123
1123	360	361
1178	1177	1176
337	359	361
1178	1173	1176
299	1124	298
128	134	127
994	981	982
57	54	55
686	688	689
254	252	263
1127	374	417
346	1180	354
435	474	1166
581	560	889
343	346	347
589	596	592
457	389	396
1126	369	313
316	317	318
343	506	315
558	561	560
29	24	22
408	316	303
447	472	473];

    % define colors for each face    
    mywhite = [0.9,0.9,0.8];
    
    colors = mywhite .* ones(height(F),1);
end

