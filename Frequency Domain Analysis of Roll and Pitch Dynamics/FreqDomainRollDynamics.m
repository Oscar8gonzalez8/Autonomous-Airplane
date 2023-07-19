clear all
close all
clc

%% import the data
[matlabFile,path] = uigetfile({'*.xlsx;*.csv'}, ...
    'Select The Flight Simulator data');
sim = readtable([path,matlabFile]);

% extract the data
t = sim.Time_s;%Time (s)
dt_avg = mean(t(2:end)-t(1:end-1)); %(s) average time-step
N = length(t); %Number of data points
xG = sim.xRate_rad_s; %(rad/s) x-axis gyrometer angular velocity
delta_a = sim.delta_a; %[-1,1] aileron command

%% Get the frequency domain information for the Bode Plot From Data
U = fft(delta_a); %Discrete Fourier Transform of Input
Y = fft(xG); %Discrete Fourier Transform of Output
Z = Y./U; %Frequency domain transfer function
%ZMag = sqrt(real(Z).^2+imag(Z).^2); %Transfer function magnitude
ZMag = abs(Z); %Transfer function magnitude: ZMag = sqrt(real(Z).^2+imag(Z).^2);
ZPhase = angle(Z); %Transfer function Phase angle: ZPhase = atan2(imag(Z),real(Z));
Nby2 = floor(N/2); %Get half the length of the data
Mag = ZMag(1:Nby2); %Get the Bode plot Mag data
Phase = ZPhase(1:Nby2); %Get the Bode plot Phase data
df = 1/(N*dt_avg); %Get the frequency step-size
f = 0:df:df*(Nby2-1); %(Hz) get the frequency array

%% Get the frequency domain information for the Bode plot from Sys Id
s = 1i*2*pi*f; %s = j*w
b0 = 30; %Guess the numerator coefficient (Your assignment is to find this!)
a0 = 8; %Guess the denominator coefficient (Your assignment is to find this!)
TFsys = b0 ./ (s+a0); %Get the transfer function
Magsys = abs(TFsys); %Get the transfer function Mag
Phasesys = angle(TFsys); %Get the transfer function Phase

%% Plot the Bode plot from data versus the Bode plot from Sys Id
figure
subplot(211)
semilogx(f,20*log10(Mag), f, 20*log10(Magsys),'--')
title('Bode Plot')
ylabel('20log_1_0(Mag)')
xlimits = [0.1,10];
xlim(xlimits)
legend('Data','Model')
grid on
subplot(212)
semilogx(f,Phase*180/pi, f, Phasesys*180/pi,'--')
ylabel('Phase (deg)')
xlabel('Frequency (Hz)')
grid on
xlim(xlimits)
legend('Data','Model')

%% Run a time-domain simulation to compare the model with the data
A = -a0;
B = 1;
C = b0;
Fd = expm([A*dt_avg,B*dt_avg;0,0]);
Ad = Fd(1,1);
Bd = Fd(1,2);
y = zeros(N,1);
x = 0;
for ii = 1:N
    y(ii) = C*x;
    x = Ad*x+Bd*delta_a(ii);
end

figure
plot(t,xG, t, y,'--')
legend('Data','Model')
xlabel('Time (s)')
ylabel('Roll Rate (rad/s)')
title('Model Prediction of Roll Rate')
xlim([0,20])