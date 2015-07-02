function [ dt_leg, dt_arm ] = walking_pattern_generatorODE( xi_Fi_ref, xi_Hi_ref,...
    xi_B_ref, ddz_G,x_c, y_c,z_c,T_Cx,T_Cy)
%WALKING_PATTERN_GENERATOR calculate the leg and arm velocities given
%references velocities of the hands, feet and waist and acceleration of the
%center of gravity of the robot

%   eps_Fi_ref : velocity (linear and angular) of both feet
%   eps_Hi_ref : velocity (linear and angular) of both hands
%   eps_B_ref : velocity (linear and angular) of the waist
%   ddz_G : acceleration of the COG of the robot
%   lambda_ref : contact forces scalar

%   dt_leg : joint velocities of the legs
%   dt_arm : joint velocities of the arms

%% Global variables
global epsilon;
global max_v_arm;
global max_v_leg;
global Lk0;
global Lk1;
global dx_G1;
global dy_G1;
global dz_G1;
global dz_G0;
global dt;
global zG; %TODO: this will stop being global if ddz_G ~= 0
global iteration;
global limit_vel;
global P_B_COG;
global dL;
%% Initialisation of the velocity of the waist
% It is first initialised to a high value to start the algorithm corectly
v_B = [1000;1000;1000];
first_time = 1;

%% Walking Pattern Generator loop
% do the following steps while |v_B_ref-v_B|>epsilon
while abs(xi_B_ref(1:3)-v_B)>epsilon
    if first_time == 0
        % exchange the linear velocity given in reference with the new one found in
        % previous iteration
        abs(xi_B_ref(1:3)-v_B);
        xi_B_ref(1:3) = v_B;
        iteration
    else
        first_time = 0;
    end
    % get the angular velocities of the arm and the legs
    [dt_leg, dt_arm] = reference_angular_momentum(xi_B_ref, xi_Fi_ref, xi_Hi_ref);
    if limit_vel == 1
        dt_leg(:,1) = velocity_check(dt_leg(:,1),max_v_leg);
        dt_leg(:,2) = velocity_check(dt_leg(:,2),max_v_leg);
        dt_arm(:,1) = velocity_check(dt_arm(:,1),max_v_arm);
        dt_arm(:,2) = velocity_check(dt_arm(:,2),max_v_arm);
    end
    % extract the vector of joint velocities
    dtheta = [dt_leg(:,1); dt_leg(:,2); dt_arm(:,1); dt_arm(:,2)];
    
    
    % find L_ref, angular momentum of the robot
    Lk1 = angular_momentum(xi_B_ref, dtheta);
    % differentiate it to get dL_ref
    dL_ref = differentiate(Lk1, Lk0);
    
   dL(:,iteration)=dL_ref;
    % calculate the x and y position of the COG in next time iteration by checking the CWS
    % stability criterion
    [x, y] = moment_equationsODE(zG,ddz_G, dL,x_c, y_c,z_c,T_Cx,T_Cy);
   
    % differentiate the acceleration to get the velocities
%     if iteration~=1
%    dx_G1 = differentiate(x(iteration,1), x(iteration-1,1));
%    dy_G1 = differentiate(y(iteration,1), y(iteration-1,1));
%     else
%       dx_G1 = 0;
%      dy_G1 =0;
%     end
%       dx_G1 = differentiate(x_Gnext,p_G(1));
%       dy_G1 = differentiate(y_Gnext,p_G(2));

    
    %integrate the acceleration on z to get velocity
    dz_G1 = integrate(ddz_G(iteration), dz_G0);
    %TODO: how to deal with the next values of z_G
    %z_Gnext= integrate(dz_G1, z_Gprev); 
    
    % calculate P_ref, momentum of the robot
   P_ref = momentum([x(iteration,2); y(iteration,2); dz_G1]);
%      P_ref = momentum([dx_G1; dy_G1; dz_G1]);
    % find the new waist velocity
    xi_B = resolved_momentum_control(P_ref,Lk1,xi_Fi_ref,xi_Hi_ref);
    
    % multiply the velocity with dt
   %xi_B =xi_B_ref+ xi_B*dt;%FIXME: why multiply by dt?
    
    % extract the linear velocity
    v_B = xi_B(1:3);
    
  
end

%% Final results when the velocities are correct
[dt_leg, dt_arm] = reference_angular_momentum(xi_B_ref, xi_Fi_ref, xi_Hi_ref);
if limit_vel == 1
    dt_leg(:,1) = velocity_check(dt_leg(:,1),max_v_leg);
    dt_leg(:,2) = velocity_check(dt_leg(:,2),max_v_leg);
    dt_arm(:,1) = velocity_check(dt_arm(:,1),max_v_arm);
    dt_arm(:,2) = velocity_check(dt_arm(:,2),max_v_arm);
    %TODO: warnings for when an "optimal" value exceeds the vel limits
end

% set the new angular momentum of the robot
Lk0 = Lk1;
dz_G0 = dz_G1;


end

