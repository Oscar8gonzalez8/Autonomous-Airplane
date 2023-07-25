clear
clc

qPrev = [3;-1;-2;7] / norm([3;-1;-2;7]);
q = function_QuaternionEstimator(qPrev,[4;-6;-1], [0.6;7;4], [1.1;4.2;-3],7,41,0.9,0.8);
qTrue = [-0.01037161520127809211100355923918
 0.93837989234331653509713078165078
 0.23893648383905119469972078150022
 0.24948940645745487643125670729205];

if abs(q(1) - qTrue(1)) > 0.00000001
    disp('e0 is not correct')
elseif abs(q(2) - qTrue(2)) > 0.00000001
    disp('e1 is not correct')
elseif abs(q(3) - qTrue(3)) > 0.00000001
    disp('e2 is not correct')
elseif abs(q(4) - qTrue(4)) > 0.00000001
    disp('e3 is not correct')
else
    disp('Everything is correct')
end


