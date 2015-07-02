clc;
close all;
%clear all;
run('init_nao_parameters.m');

%% Global variables
global steptime;
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


global P_COG;
global zG;
global dP_COG;
global com_measured;
global com_desired;

global start;
global limit_vel;

global max_height;
global mid_height;
global lambda_coeff;

global Alpha;
global Kp;
global Ki;
global Kd;

global step_time step_length
global animation;
global Ts;

%% Parameters for the simulation
com_measured = zeros(3,nb_points*nb_steps+1);
start = 1; % not to be modified
traj_time = zeros(1,nb_points*nb_steps+1);

pid_vel = 0;
limit_vel = 0;
plot_res = 1;
animation = 0;

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
nb_points = 100;
nb_steps = 6;
swing_foot = 2; % not to be modified
step_time = .5;
step_length = 50*10^-3;
max_height = 20*10^-3;
mid_height = 2*10^-3;
lambda_coeff = 0.25;
Ts=step_time*nb_steps;
%%Initialisation of global reference transformation matrix
T_global=eye(4);

%% Initialisation of the time step
dt = step_time/(nb_points); % not to be modified 

%% Initialisation of the time for the animation
steptime = dt; % not to be modified

%% Initialisation of the reference velocities

% velocity of both feet
%xi_Fi_traj = zeros(6,2,nb_points*nb_steps+1);
% velocity of both hands
xi_Hi_traj = zeros(6,2,nb_points*nb_steps+1);
% velocity of the waist
xi_B_traj = zeros(6,nb_points*nb_steps+1);
for plop=2:nb_points*nb_steps+1
%     if mod(plop,2)==0
%     xi_B_traj(3,plop)=.5;
%     else
%         %xi_B_traj(1,plop)=1;
%     end
xi_B_traj(1,plop)=step_length/step_time ;
    %xi_B_traj(2,plop)=.15*sin(plop*dt*pi*2);
 xi_Hi_traj(1,1,plop) =step_length/step_time ;
 xi_Hi_traj(1,2,plop) =step_length/step_time ;
 %xi_B_traj(3,plop)=.05;
end
%initialize with values obtain from COG solved without angularmomentum
%derivtive
% load inx;
% load iny;
% for plop=2:nb_points*nb_steps+1
%     xi_B_traj(1,plop)=x(plop,2);
%     xi_B_traj(2,plop)=y(plop,2);;
%  xi_Hi_traj(1,1,plop) =x(plop,2);
%  xi_Hi_traj(1,2,plop) =x(plop,2);
% end

%initialize with values obtain from resolved momentum first iterqtion 
% load initvB;
% 
%     xi_B_traj(2,:)=v_B(2,:);
%     xi_B_traj(1:3,:)=v_B;
%      xi_Hi_traj(1,1,:) =v_B(1,:);
%   xi_Hi_traj(1,2,:) =v_B(1,:);

% velocity of the COG
dx_G0 = 0;
dy_G0 = 0;
dz_G0 = 0;

%% Initialisation of the accelerations
ddz_G = zeros(1,nb_points*nb_steps+1);

%% Initialisation of the starting position
theta_arm = zeros(4,2,nb_points*nb_steps+1);
theta_leg = zeros(6,2,nb_points*nb_steps+1);

run('init_robot_position.m');

t_arm_0 = theta_arm(:,:,1);
t_leg_0 = theta_leg(:,:,1);

t_arm_init = theta_arm(:,:,1);
t_leg_init = theta_leg(:,:,1);


%% Move the robot to its initial position
[chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
    move_robot(theta_leg,theta_arm,theta_head,swing_foot,swing_foot,1);
T_leg_R = chain_leg_R(:,:,end);
T_leg_L = chain_leg_L(:,:,end);
% animate the motion
if animation == 1
    animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
end

%% Initialisation of the height of the ground
z0 = 0;
zG = P_COG(3,1);
dP_COG = zeros(3,1);

%% Vectors for checking
real_pos = zeros(3,2,nb_steps*nb_points);
real_pos_arm = zeros(3,2,nb_steps*nb_points);
desired_pos = zeros(3,2,nb_steps*nb_points);

feet_state = [chain_leg_R(1:3,4,end), chain_leg_L(1:3,4,end)];
hands_state = [chain_arm_R(1:3,4,end), chain_arm_L(1:3,4,end)];

com_traj = zeros(3,2,nb_steps*nb_points);
[P_F,p_k,xi_Fi_traj,lambda_traj, n_k,swing]= trajectory_generation(nb_steps,nb_points,...
                    step_length,step_time,T_leg_R,T_leg_L,max_height,mid_height);
                
%                for plop=2:nb_points*nb_steps+1
%                     if mod(plop,2)==0
%                         xi_B_traj(3,plop)=1;
% %                         xi_Fi_traj(5,2,3)=10;
%                    else
%                         xi_B_traj(3,plop)=1;
%                    end
% %                     xi_B_traj(1,:)=xi_Fi_traj(1,swing,:);
% %                     xi_Hi_traj(1,1,:) =xi_Fi_traj(1,swing,:) ;
% %                     xi_Hi_traj(1,2,:) =xi_Fi_traj(1,swing,:) ;
%                     
%                end
%  [theta_arm,theta_leg] = TestJacob (ddz_G, lambda_traj,p_k, alpha, n_k,t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG,theta_head,swing)
test=0;
if test==0;
%     %see alg behaviour from last result
%      load VB
%     xi_B_traj(1:3,:) = v_B;
%     % xi_B_traj(1:2,:) = v_B(1:2,:);
%     xi_Hi_traj(1:2,1,:) =v_B(1:2,:);
%     xi_Hi_traj(1:2,2,:) =v_B(1:2,:);
%     % end of last result preparation
[theta_arm,theta_leg] = WalkingPatternGenerator (ddz_G, lambda_traj,p_k,...
    alpha, n_k,t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG,theta_head,swing);

 savefile='results/thArm.mat';
    save(savefile,'theta_arm');
     savefile='results/thLeg.mat';
    save(savefile,'theta_leg');
writeOutputFile(theta_arm,theta_leg);
else
    
    load results/thArm
    load results/thLeg
    
    load results/VB
    xi_B_traj(1:3,:) = v_B;
    xi_Hi_traj(1:2,1,:) =v_B(1:2,:);
    xi_Hi_traj(1:2,2,:) =v_B(1:2,:);
    
    testRestults(ddz_G, lambda_traj,p_k, alpha, n_k,theta_arm,theta_leg,...
        xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG,theta_head,swing)
end