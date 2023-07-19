function q = function_RI2b_to_Quaternion(R)
% q = function_RI2b_to_Quaternion(R)
% This function calculates the quaternion orientation
% e0, e1, e2, and e3 from a known inertial to body
% rotation matrix R from the inertial to body frame
%calculate the trace of the rotation matrix
t = R(1,1) + R(2,2) + R(3,3);
if t > 0 %the trace is positive, no chance of imaginary numbers
s = sqrt(t+1)*2; % calculate an intermediate parameter
e0 = 0.25 * s; %get e0
e1 = (R(2,3)-R(3,2))/s; %get e1
e2 = (R(3,1)-R(1,3))/s; %get e2
e3 = (R(1,2)-R(2,1))/s; %get e3
elseif R(1,1) > max(R(2,2), R(3,3))
%The trace is negative...Avoid imaginary numbers
s = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;
e0 = (R(2,3)-R(3,2))/s; %get e0
e1 = 0.25 * s; %get e1
e2 = (R(1,2)+R(2,1))/s; %get e2
e3 = (R(1,3)+R(3,1))/s; %get e3
elseif R(2,2) > max(R(1,1), R(3,3))
%The trace is negative...Avoid imaginary numbers
s = sqrt(1-R(1,1)+R(2,2)-R(3,3))*2;
e0 = (R(3,1)-R(1,3))/s; %get e0
e1 = (R(1,2)+R(2,1))/s; %get e1
e2 = 0.25 * s; %get e2
e3 = (R(2,3)+R(3,2))/s; %get e3
else
%The trace is negative...Avoid imaginary numbers
s = sqrt(1-R(1,1)-R(2,2)+R(3,3))*2;
e0 = (R(1,2)-R(2,1))/s; %get e0
e1 = (R(1,3)+R(3,1))/s; %get e1
e2 = (R(2,3)+R(3,2))/s; %get e2
e3 = 0.25 * s; %get e3
end
q = [e0;e1;e2;e3];

end