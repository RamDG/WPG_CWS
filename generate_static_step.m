function [ P_Fi, P_SR, P_SL, eps_Fi, lambda] = generate_static_step( r, alpha,...
    T, nb_points, swing_foot)
%GENERATE_STEP generate a trajectory for the foot over one step
%   r : distance from origine pose to goal
%   alpha : angle (polar coordinates)
%   psi : stearing angle
%   t : time of the step
%   foot : value for the foot (1 for right, 2 for left)

%   P_F : position of the center of the feet
%   P_S : position of the four sensors
%   eps_F : linear and angular velocities

%% Global variables
global T_leg_R;
global T_leg_L;
global P_B_COG;
global dP_B_COG;
global dt;

global lambda_coeff;
global max_height;
global mid_height;

%% Parameters for the trajectory
time = T/10;
step = nb_points/10;

%% Generate the trajectory
% initialisation
P_Fi = zeros(3,2,nb_points);
P_SR = zeros(3,4,nb_points);
P_SL = zeros(3,4,nb_points);
eps_Fi = zeros(6,2,nb_points);

% time vector*
t = linspace(0,T,nb_points);
t_vec = linspace(0,T-2*time,nb_points-2*step);

%% Initialise the position
for i = 1:nb_points
    P_Fi(:,1,i) = T_leg_R(1:3,4);
    P_Fi(:,2,i) = T_leg_L(1:3,4);
end


%% Generate the velocity of the feet
% use the central difference to calculate the velocity
for i = 2:(nb_points-1)
    eps_Fi(1:3,:,i) = (P_Fi(:,:,i+1) - P_Fi(:,:,i-1))/(2*dt);
end

% get the position of the foot sensors
for i = 1:nb_points
    % create the transformation matrix with the position of the foot
    T_R = makehgtform('translate',P_Fi(:,1,i));
    T_L = makehgtform('translate',P_Fi(:,2,i));
    temp_ps = get_sensors_position(T_R, T_L);
    P_SR(:,:,i) = temp_ps(:,1,:);
    P_SL(:,:,i) = temp_ps(:,2,:);
end

%% Creation of the vertices
vertix = [P_SR, P_SL];
lambda = zeros(8,nb_points); % 4 contacts on each feet

    lambda(:,:) = ones(8,nb_points)*lambda_coeff;
    


end
