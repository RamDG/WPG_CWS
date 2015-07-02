clc;
close all;

run('init_nao_parameters.m');

%% Global variables
global steptime;
global iteration;
global epsilon;
global dt;
global Lk0;
global Lk1;
global dx_G0;
global dy_G0;
global dz_G0;
global g;
global z0;
global T_leg_R;
global T_leg_L;
global joint_names;
global T_global;

global chain_leg_R;
global chain_leg_L;
global chain_arm_R;
global chain_arm_L;
global chain_head;

global P_SR;
global P_SL;
global P_B_COG;
global zG;
global dP_B_COG;
global com_measured;
global com_desired;

global start;
global limit_vel;

global max_height;
global mid_height;
global lambda_coeff;
global total_mass;
global swing_foot;

global Alpha;
global Kp;
global Ki;
global Kd;

%% Initialisation of the gravity constant
g = 9.81;

%% Initialisation of parameters for the PID
Alpha = 1;
Ki = 5;

%% Initiliastion of epsilon, threshold to stop the loop
epsilon = 0.0001;

%% Parameter for criterion
alpha=0;

%% Initialisation of the angular momentum of the robot
Lk0 = zeros(3,1); % not to be modified
Lk1 = zeros(3,1); % not to be modified

%% Parameters of the trajectory
nb_points = 20;
nb_steps =10 ;
swing_foot = 2; % not to be modified
step_time = 1;
step_length = 50*10^-3;
max_height = 10*10^-3;
mid_height = 2*10^-3;
lambda_coeff = 0.25;

%%Initialisation of global reference transformation matrix
T_global=eye(4);

%% Initialisation of the time step
dt = step_time/(nb_points-1); % not to be modified (-1 to consider initial 0)

%% Initialisation of the time for the animation
steptime = dt; % not to be modified

%% Initialisation of the reference velocities
%TODO: eps_ variables should be called xi_ double check the variable in the
%theory (not so important but easier to compare with theory)
% velocity of both feet
xi_Fi_traj = zeros(6,2,nb_points);
% velocity of both hands
xi_Hi_traj = zeros(6,2,nb_points);
% velocity of the waist
xi_B_traj = zeros(6,1,nb_points);
for plop=2:nb_points-1
    xi_B_traj(1,1,plop)=[step_length/step_time];
    xi_B_traj(2,1,plop)=.5*sin(plop*dt*pi*2);
end
% velocity of the COG
dx_G0 = 0;
dy_G0 = 0;
dz_G0 = 0;

%% Initialisation of the accelerations
ddz_G_traj = zeros(1,1,nb_points);

%% Initialisation of the starting position
theta_arm = zeros(4,2,nb_points*nb_steps);
theta_leg = zeros(6,2,nb_points*nb_steps);

run('init_robot_position.m');

t_arm_0 = theta_arm(:,:,1);
t_leg_0 = theta_leg(:,:,1);

%% Parameters for the simulation
com_measured = zeros(3,nb_points*nb_steps);
start = 1; % not to be modified
iteration = 1; % not to be modified
traj_time = zeros(1,nb_points*nb_steps);

pid_vel = 0;
limit_vel = 0;
plot_res = 1;
animation = 1;

%% Move the robot to its initial position
[chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
    move_robot(theta_leg,theta_arm,theta_head);
% animate the motion
if animation == 1
    animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
end

%% Initialisation of the height of the ground
z0 = 0;
zG = P_B_COG(3,1);
dP_B_COG = zeros(3,1);

%% Vectors for checking
real_pos = zeros(3,2,nb_steps*nb_points);
real_pos_arm = zeros(3,2,nb_steps*nb_points);
desired_pos = zeros(3,2,nb_steps*nb_points);

feet_state = [chain_leg_R(1:3,4,end), chain_leg_L(1:3,4,end)];
hands_state = [chain_arm_R(1:3,4,end), chain_arm_L(1:3,4,end)];

com_traj = zeros(3,2,nb_steps*nb_points);
%% Walking along the trajectory
for s = 1:nb_steps
    
    % generate the trajectory for one step
    if start == 1
        % get the transformation matrix for the legs
        T_leg_R = chain_leg_R(:,:,end);
        T_leg_L = chain_leg_L(:,:,end);
        
        [P_F, P_SR_traj, P_SL_traj, xi_Fi_traj, lambda_traj] = ...
            generate_stepNoBck(step_length/2,0,step_time,nb_points,...
            swing_foot);
%         [P_F, P_SR_traj, P_SL_traj, eps_Fi_traj, lambda_traj] = ...
%             generate_static_step(step_length,0,step_time,nb_points,...
%             swing_foot);
        start = 0;
        %com_desired = com_traj;
        desired_pos(:,:,(s-1)*nb_points+1:s*nb_points) = P_F;
    else
        % just for testing get the last value of previous trajectory to be
        % the new starting point for the next step
%         T_leg_R = makehgtform('translate',P_F(:,1,end));
%         T_leg_L = makehgtform('translate',P_F(:,2,end));
                T_leg_R = chain_leg_R(:,:,end);
                T_leg_L = chain_leg_L(:,:,end);
        
        [P_F, P_SR_traj, P_SL_traj, xi_Fi_traj, lambda_traj] = ...
            generate_stepNoBck(step_length,0,step_time,nb_points,...
            swing_foot);
%         [P_F, P_SR_traj, P_SL_traj, eps_Fi_traj, lambda_traj] = ...
%             generate_static_step(step_length,0,step_time,nb_points,...
%             swing_foot);
       % com_desired = [com_desired, com_traj];
        desired_pos(:,:,(s-1)*nb_points+1:s*nb_points) = P_F;
    end
    
    % reset the parameters between two steps
    dx_G0 = 0;
    dy_G0 = 0;
    dz_G0 = 0;
    Lk0 = zeros(3,1);
    Lk1 = zeros(3,1);
    
    for t = 1:nb_points
        % extract the parameters at the time t
        xi_Fi = xi_Fi_traj(:,:,t);
        xi_Hi = xi_Hi_traj(:,:,t);
        xi_B = xi_B_traj(:,:,t);
        ddz_G = ddz_G_traj(:,:,t);
        lambda_ref = lambda_traj(:,t)';
        P_SR = P_SR_traj(:,:,t);
        P_SL = P_SL_traj(:,:,t);
        %TODO: how to get n_k in a more general case (related to unit
        %normal vector at each contact point
        n_k=[zeros(1,size(lambda_ref,2));zeros(1,size(lambda_ref,2));ones(1,size(lambda_ref,2))];
        %NOTE: this n_k valid only for flat floor
        
        % if we want to get the velocities using a PID
        if pid_vel == 1
            % calculate the velocities of the feet
            feet_error = desired_pos(:,:,iteration) - feet_state;
            v_F = xi_Fi(1:3,:) + Ki*feet_error;
            
            % replace the velocity of the trajectory with the calculated
            % one
            xi_Fi(1:3,:) = v_F;
        end
        
        if iteration==1
            p_k = [P_SR, P_SL];
            eps_k = zeros(1,size(lambda_ref,2));
            
            for k = 1:size(lambda_ref,2)
                eps_k(1,k) = (1-alpha)*total_mass*(ddz_G + g)*lambda_ref(1,k)...
                    /sum(lambda_ref(1,:))*n_k(3,k);
            end
            
            %% sum of forces on contacts and moments
            eps=sum(eps_k(1,:));
            x_c=0;
            y_c=0;
            T_Cx=0;
            T_Cy=0;
            
            for k = 1:size(lambda_ref,2)
                x_c= x_c + alpha * eps_k(1,k) * p_k(1,k) / eps;
                y_c= y_c + alpha * eps_k(1,k) * p_k(2,k) /eps;
                T_Cx= T_Cx + eps_k(1,k) * ( p_k(2,k) * n_k(3,k) - p_k(3,k) * n_k(2,k) );
                T_Cy= T_Cy - eps_k(1,k) * ( p_k(1,k) * n_k(3,k) - p_k(3,k) * n_k(1,k) );
                
            end
            %TODO: check if there will be any case in which dL is required
            %in the first step
            x_G= -T_Cy / (total_mass * g) + x_c;
            y_G= T_Cx / (total_mass * g) + y_c;
%             
%              com_traj(1,step) = x_G;
%              com_traj(2,step) = y_G;
%              
%              %TODO: change to consider case where there is acc in z
%              com_traj(3,step) = zG;
%             
%              % plug them in the walking pattern generator
%         [ dt_leg, dt_arm, x_Gnext, y_Gnext ] = walking_pattern_generator(eps_Fi, eps_Hi,eps_B,...
%             ddz_G, lambda_ref, alpha, n_k , com_traj(:,step) , com_traj(:,step));
        
%         %temporal tryout
        com_traj(:,iteration) = P_B_COG;
       
          [ dt_leg, dt_arm, x_Gnext, y_Gnext ] = walking_pattern_generator(xi_Fi, xi_Hi,xi_B,...
            ddz_G, lambda_ref, alpha, n_k , com_traj(:,iteration) , com_traj(:,iteration),theta_leg(:,:,iteration),theta_arm(:,:,iteration),theta_head);
        else
        
        % plug them in the walking pattern generator
        [ dt_leg, dt_arm, x_Gnext, y_Gnext ] = walking_pattern_generator(xi_Fi, xi_Hi,xi_B,...
            ddz_G, lambda_ref, alpha, n_k , com_traj(:,iteration) , com_traj(:,iteration-1),theta_leg(:,:,iteration),theta_arm(:,:,iteration),theta_head);
        end
%        com_traj(1,step+1) = x_Gnext;
%        com_traj(2,step+1) = y_Gnext;
%        
%         %TODO: change to consider case where there is acc in z
%              com_traj(3,step+1) = zG;
       
        % integrate the joint velocities to get the angles values
        theta_arm(:,:,iteration) = integrate(dt_arm, t_arm_0);
        theta_leg(:,:,iteration) = integrate(dt_leg, t_leg_0);
        
        %ensure that the angular value is between [-pi/2;pi/2]
        theta_arm(:,:,iteration) = mod_interval(theta_arm(:,:,iteration),-2*pi,2*pi);
        theta_leg(:,:,iteration) = mod_interval(theta_leg(:,:,iteration),-2*pi,2*pi);
        
        % move the robot with the new desired joint values
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot(theta_leg(:,:,iteration),theta_arm(:,:,iteration),theta_head);
        % animate the motion
        if animation == 1
            animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
        
        %FIXME: Temporary fix to use actual com
        com_traj(:,iteration+1) = P_B_COG;
        
        real_pos(:,1,iteration) = chain_leg_R(1:3,4,end);
        real_pos(:,2,iteration) = chain_leg_L(1:3,4,end);
        
        real_pos_arm(:,1,iteration) = chain_arm_R(1:3,4,end);
        real_pos_arm(:,2,iteration) = chain_arm_L(1:3,4,end);
        
        % set the new value of the joint for the integration
        t_arm_0 = theta_arm(:,:,iteration);
        t_leg_0 = theta_leg(:,:,iteration);
        
        % update the state of the system
        feet_state = [chain_leg_R(1:3,4,end), chain_leg_L(1:3,4,end)];
        hands_state = [chain_arm_R(1:3,4,end), chain_arm_L(1:3,4,end)];
        
        % collect the time stamp
        traj_time(1,iteration) = iteration*dt;
        
        % increase the step number
        iteration = iteration + 1;
    end
    
    % derivate the desired pos of the COM to get the velocity of the COM at
    % next step
    dP_B_COG = differentiate(com_traj(:,nb_points),com_traj(:,nb_points-1));
    disp('step : ');disp(s);
    % change the swing foot
    if swing_foot == 1
        swing_foot = 2;
    else
        swing_foot = 1;
    end
end

%com_diff = com_desired - com_measured;
pos_diff = desired_pos - real_pos;

pos_diff_R = zeros(3,step-1);
pos_diff_L = zeros(3,step-1);
pos_diff_R(:,:) = pos_diff(:,1,:);
pos_diff_L(:,:) = pos_diff(:,2,:);

real_pos_R = zeros(3,step-1);
real_pos_L = zeros(3,step-1);
real_pos_R(:,:) = real_pos(:,1,:);
real_pos_L(:,:) = real_pos(:,2,:);

real_pos_armR = zeros(3,step-1);
real_pos_armL = zeros(3,step-1);
real_pos_armR(:,:) = real_pos_arm(:,1,:);
real_pos_armL(:,:) = real_pos_arm(:,2,:);

des_pos_R = zeros(3,step-1);
des_pos_L = zeros(3,step-1);
des_pos_R(:,:) = desired_pos(:,1,:);
des_pos_L(:,:) = desired_pos(:,2,:);

T = step_time*nb_steps;
time_vector = linspace(0,T,nb_points*nb_steps);

if plot_res == 1
    % desired pos
    figure();
    subplot(2,3,1);
    plot(time_vector,des_pos_R(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot x');
    subplot(2,3,2);
    plot(time_vector,des_pos_R(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot y');
    subplot(2,3,3);
    plot(time_vector,des_pos_R(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot z');
    subplot(2,3,4);
    plot(time_vector,des_pos_L(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot x');
    subplot(2,3,5);
    plot(time_vector,des_pos_L(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot y');
    subplot(2,3,6);
    plot(time_vector,des_pos_L(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot z');
    set(gcf,'NextPlot','add');
    axes;
    h = suptitle('Desired position of the feet');
    set(gca,'Visible','off');
    set(h,'Visible','on'); 
  
    % real pos
    figure();
    subplot(2,3,1);
    plot(time_vector,real_pos_R(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot x');
    subplot(2,3,2);
    plot(time_vector,real_pos_R(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot y');
    subplot(2,3,3);
    plot(time_vector,real_pos_R(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right foot z');
    subplot(2,3,4);
    plot(time_vector,real_pos_L(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot x');
    subplot(2,3,5);
    plot(time_vector,real_pos_L(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot y');
    subplot(2,3,6);
    plot(time_vector,real_pos_L(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left foot z');
    set(gcf,'NextPlot','add');
    axes;
    h = suptitle('Measured position of the feet');
    set(gca,'Visible','off');
    set(h,'Visible','on'); 
    
    % real pos arm
    figure();
    subplot(2,3,1);
    plot(time_vector,real_pos_armR(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right hand x');
    subplot(2,3,2);
    plot(time_vector,real_pos_armR(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right hand y');
    subplot(2,3,3);
    plot(time_vector,real_pos_armR(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('right hand z');
    subplot(2,3,4);
    plot(time_vector,real_pos_armL(1,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left hand x');
    subplot(2,3,5);
    plot(time_vector,real_pos_armL(2,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left hand y');
    subplot(2,3,6);
    plot(time_vector,real_pos_armL(3,:));
    ylabel('meter');
    xlabel('number of steps');
    title('left hand z');
    set(gcf,'NextPlot','add');
    axes;
    h = suptitle('Measured position of the hands');
    set(gca,'Visible','off');
    set(h,'Visible','on'); 
    
%     % com position
%     figure();
%     subplot(3,3,1);
%     plot(time_vector,com_desired(1,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('x desired');
%     subplot(3,3,2);
%     plot(time_vector,com_desired(2,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('y desired');
%     subplot(3,3,3);
%     plot(time_vector,com_desired(3,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('z desired');
%     subplot(3,3,4);
%     plot(time_vector,com_measured(1,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('x measured');
%     subplot(3,3,5);
%     plot(time_vector,com_measured(2,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('y measured');
%     subplot(3,3,6);
%     plot(time_vector,com_measured(3,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('z measured');
%     subplot(3,3,7);
%     plot(time_vector,com_diff(1,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('difference in x');
%     subplot(3,3,8);
%     plot(time_vector,com_diff(2,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('difference in y');
%     subplot(3,3,9);
%     plot(time_vector,com_diff(3,:));
%     ylabel('meter');
%     xlabel('number of steps');
%     title('difference in z');
%     
%     set(gcf,'NextPlot','add');
%     axes;
%     h = suptitle('Position of the Center of Mass');
%     set(gca,'Visible','off');
%     set(h,'Visible','on');
end

%% Write the trajectory of the joints as a csv file
% first combine leg and arm trajectories starting with the leg
% theta = zeros(nb_points*nb_steps,length(joint_names));
theta = zeros(nb_points*nb_steps,24);

% the size of the right leg is one less than the left one as the hip joint
% is the same
theta_leg_R = zeros(size(theta_leg,1)-1,nb_points*nb_steps);
theta_leg_L = zeros(size(theta_leg,1),nb_points*nb_steps);
theta_arm_R = zeros(size(theta_arm,1),nb_points*nb_steps);
theta_arm_L = zeros(size(theta_arm,1),nb_points*nb_steps);

% add zeros values for the head and the hands angles
theta_head = zeros(2,nb_points*nb_steps);
theta_hand_R = zeros(2,nb_points*nb_steps);
theta_hand_L = zeros(2,nb_points*nb_steps);

% remove the first value of the right leg
theta_leg_R(:,:) = theta_leg(2:end,1,:);
theta_leg_L(:,:) = theta_leg(:,2,:);

theta_arm_R(:,:) = theta_arm(:,1,:);
theta_arm_L(:,:) = theta_arm(:,2,:);

% theta(:,:) = [theta_head',...
%     theta_arm_L',theta_hand_L',...
%     zeros(nb_points*nb_steps,1),theta_leg_L(2:end,:)',...
%     zeros(nb_points*nb_steps,1),theta_leg_R',...
%     theta_arm_R',theta_hand_R',traj_time'];
%
% csvwrite_with_headers('trajectory.csv',theta,joint_names);

theta(:,:) = [zeros(nb_points*nb_steps,1),theta_leg_R',...
    zeros(nb_points*nb_steps,1),theta_leg_L(2:end,:)',...
    theta_arm_R',zeros(nb_points*nb_steps,1),...
    theta_arm_L',zeros(nb_points*nb_steps,1),...
    theta_head'];

theta = filtfilt(ones(1,5)/5,1,theta);
csvwrite('trajectory.csv',theta');

