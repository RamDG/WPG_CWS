function [ P ] = momentum( dX_G )
%MOMENTUM calculate the momentum of the robot
%   dX_G : vector of acceleration of the COG along x,y and z axis 

%   P : momentum of the robot, 3X1 vector

%% Global variables
global total_mass;

%% Calculation of the momentum
P = total_mass*dX_G;

end