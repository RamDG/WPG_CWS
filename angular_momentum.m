function [ L ] = angular_momentum( xi_B, dtheta )
%ANGULAR_MOMENTUM calculate the angular momentum of the robot given the
%velocities of the waist and the angular velocities of the joints
%   esp_B : velocities of the waist
%   dtheta : joint velocities
%   Mt :
%   Ht :

%   L : angular momentum of the robot

%% Global variables
global r_B_COG;
global total_inertia;
global total_mass;
global Mt;
global Ht;

%% Calculation of the momentum
mat = [total_mass*eye(3,3), -total_mass*skew(r_B_COG), Mt;...
    zeros(3,3), total_inertia, Ht];

momentum = mat*[xi_B; dtheta];

%% Extraction of the angular momentum
L = momentum(4:6,1);

end

