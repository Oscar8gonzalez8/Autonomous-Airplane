clear all;
%close all;
clf
s = tf('s');
a = 1000
b = 100; 
sysD = b / (s + a);
w=logspace(-5,5);
[mag,ph]=bode(sysD,w);
subplot(2,1,1)
loglog(w,squeeze(mag),'LineWidth',2);
grid on;

title('Problem 6.16 (a) Magnitude');
xlabel('\omega (rad/sec)');
ylabel('Magnitude');
subplot(2,1,2)
semilogx(w,squeeze(ph),'LineWidth',2);
grid on;

title('(b) Phase');
ylabel('\phi (deg)');
xlabel('\omega (rad/sec)')