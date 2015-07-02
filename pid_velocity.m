function [ v, w ] = pid_velocity( pos_error, orient_error )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

%% Global variables for the PID
global Kp;
global Ki;
global Kd;
global alpha;
global epsilon;
global dt;
global threshold;
global start;

%% Calculate the gain for the position error
normE = norm(pos_error);
if normE > epsilon
    K = (1 - exp(alpha*normE*normE))/normE;
else
    K = 100000;
end

v = -K*pos_error;

end

