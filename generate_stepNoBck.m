function [ P_Fi, P_SR, P_SL, xi_Fi, lambda] = generate_stepNoBck( r, alpha,...
    T, nb_points, swing_foot,T_leg_R,T_leg_L,max_height,mid_height)
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
global dt;
global lambda_coeff;

%% Parameters for the trajectory
time = T/10;
step = nb_points/10;
r=r*2;
ds=.2;%percentage of step in double support
ss=1-ds;%percentage of step in single support
%% Generate the trajectory
% initialisation
P_Fi = zeros(3,2,nb_points);
P_SR = zeros(3,4,nb_points);
P_SL = zeros(3,4,nb_points);
xi_Fi = zeros(6,2,nb_points);

% time vector*
t = linspace(dt,T,nb_points);
t_vec = linspace(dt,T-2*time,nb_points-2*step);

%% Initialise the position
for i = 1:nb_points
    P_Fi(:,1,i) = T_leg_R(1:3,4);
    P_Fi(:,2,i) = T_leg_L(1:3,4);
end

% move up
t_up = linspace(dt,time,step);
for i = 1:step
    P_Fi(3,swing_foot,i) = P_Fi(3,swing_foot,i) + mid_height/time*t_up(i);
end

% move forward
for i = step+1:nb_points-step
    P_Fi(1,swing_foot,i) = P_Fi(1,swing_foot,i) ...
        + (r/(T-2*time))*t_vec(i-step)*cos(alpha);
    P_Fi(2,swing_foot,i) = P_Fi(2,swing_foot,i) ...
        + (r/(T-2*time))*t_vec(i-step)*sin(alpha);
    % calculate the z coordinates
    R = (r/(T-2*time))*t_vec(i-step);
    a = -(mid_height-max_height)/(r*r/4);
    b = r/2;
    
    P_Fi(3,swing_foot,i) =  P_Fi(3,swing_foot,i) ...
        -a*(R-b).*(R-b) + max_height;
end

% move down
for i = nb_points-step+1:nb_points
    P_Fi(1,swing_foot,i) = P_Fi(1,swing_foot,i) + ...
        r*cos(alpha);
    P_Fi(2,swing_foot,i) = P_Fi(2,swing_foot,i) + ...
        r*sin(alpha);
    t_down = linspace(dt,time,step);
    P_Fi(3,swing_foot,i) = P_Fi(3,swing_foot,i) + ...
        mid_height-mid_height/time*t_down(i-nb_points+step);
end



%% Generate the velocity of the feet
% use the central difference to calculate the velocity
for i = 2:(nb_points-1)
    xi_Fi(1:3,:,i) = (P_Fi(:,:,i+1) - P_Fi(:,:,i-1))/(2*dt);
end
figure,
   plot(squeeze( xi_Fi(1,swing_foot,:)),'r'); hold on;
    plot(squeeze( xi_Fi(2,swing_foot,:))); hold on;
    plot(squeeze( xi_Fi(3,swing_foot,:)),'g'); hold on;
    legend('xi_{Fx}','xi_{Fy}','xi_{Fz}',3);
      xlabel('1 step');
      ylabel('Linear Velocity, m/s')
% 
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

if swing_foot == 1
    lambda(5:8,:) = ones(4,nb_points)*lambda_coeff;
%     % al contacts have force in double support
%     lambda(1:4,1) = ones(4,1)*lambda_coeff;
%     lambda(1:4,nb_points) = ones(4,1)*lambda_coeff;
else
    lambda(1:4,:) = ones(4,nb_points)*lambda_coeff;
%     % al contacts have force in double support
%     lambda(5:8,1) = ones(4,1)*lambda_coeff;
%     lambda(5:8,nb_points) = ones(4,1)*lambda_coeff;
end



end
