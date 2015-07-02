function [ dXk ] = differentiate( Xk1, Xk0 )
%DIFFERENTIATE calculate the derivative of the variable X
%   Xk1 : vector to differentiate at time k
%   Xk0 : vector to differentiate at time k-1

%   dXk : the derivative at time k

%% Global variable of the time step
global dt;

%% Calculation of the derivative
dXk = (Xk1 - Xk0)/dt;

end