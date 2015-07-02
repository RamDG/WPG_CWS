function [ Xk1 ] = integrate( dXk, Xk0 )
%INTEGRATE give the value of the integral of the vector dX at time k
%   dXk : value of the derivative at time k
%   Xk0 : value of the vector at time k-1

%   Xk1 : value of the vector at time k

%% Global variable of the time step
global dt;

%% Calculation of the derivative
Xk1 = dt*dXk + Xk0;

end

