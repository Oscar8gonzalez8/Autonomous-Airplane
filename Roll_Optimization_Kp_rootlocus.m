s = tf('s');
a = 9;
b = 33;
zeta = 0.8; 
kp = 0.61;
%rltool(b*s/(s^2 + a*s + kp*b));

controlSystemDesigner(tf(b*s)/(s^2 + a*s + kp*b))