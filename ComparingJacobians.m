clc;
close all;
%clear all;
run('init_nao_parameters.m');

%% Global variables
global steptime;
global epsilon;
global dt;

global g;
global z0;
global T_leg_R;
global T_leg_L;
global T_global;

global P_COG;
global zG;

global start;
global limit_vel;

global max_height;
global mid_height;
global lambda_coeff;

global Alpha;

global step_time step_length swing_foot
global animation;

global count
count=1;
%% Parameters for the simulation
start = 1; % not to be modified

pid_vel = 0;
limit_vel = 0;
plot_res = 1;
animation = 1;

%% Initialisation of the gravity constant
g = 9.81;

%% Initialisation of parameters for the PID
Alpha = 1;
Ki = 5;

%% Initiliastion of epsilon, threshold to stop the loop
epsilon = 0.0001;

%% Parameter for criterion
alpha=0;


%% Parameters of the trajectory
nb_points = 200;
nb_steps = 1;
swing_foot = 1; % not to be modified
step_time = 2;
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

%% Initialisation of the starting position
theta_arm = zeros(4,2,nb_points*nb_steps+1);
theta_leg = zeros(6,2,nb_points*nb_steps+1);

run('init_robot_position.m');

t_arm_0 = theta_arm(:,:,1);
t_leg_0 = theta_leg(:,:,1);

t_arm_init = theta_arm(:,:,1);
t_leg_init = theta_leg(:,:,1);


%% Move the robot to its initial position
global separateChains
separateChains=0;
if separateChains==1
    [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
        move_robot(t_leg_init,t_arm_init,theta_head,swing_foot,swing_foot,1);
else
    [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
        move_robot2(t_leg_init,t_arm_init,theta_head,swing_foot,swing_foot,1);
end
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


[P_F,p_k,xi_Fi_traj,lambda_traj, n_k,swing]= trajectory_generation(nb_steps,nb_points,...
                    step_length,step_time,T_leg_R,T_leg_L,max_height,mid_height);
            %   swing=ones(1,nb_points)*2; 
               
               %% Initialisation of the reference velocities

% %velocity of both feet
% xi_Fi_traj = zeros(6,2,nb_points*nb_steps+1);
% %velocity of both hands
xi_Hi_traj = zeros(6,2,nb_points*nb_steps+1);
%velocity of the waist
xi_B_traj = zeros(6,nb_points*nb_steps+1);
for plop=2:nb_points*nb_steps+1
    if mod(plop,2)==0
    xi_B_traj(3,plop)=.5;
    else
        xi_B_traj(3,plop)=-.5;
    end
xi_B_traj(1,plop)=step_length/step_time ;
    %xi_B_traj(2,plop)=.15*sin(plop*dt*pi*2);
 xi_Hi_traj(1,1,plop) =step_length/(step_time*2) ;
 xi_Hi_traj(1,2,plop) =step_length/(step_time*2) ;
 %xi_B_traj(3,plop)=.05;
end


for plop=2:nb_points*nb_steps+1
    
    xi_B_traj(1,plop)=.3*cos(2*pi*mod(plop,50)/50);
     xi_B_traj(2,plop)=.3*cos(2*pi*mod(plop,50)/50);
     xi_Fi_traj(6,1,plop)=.8*cos(2*pi*mod(plop,50)/50);
    %                     xi_B_traj(1,:)=xi_Fi_traj(1,swing,:);
    %                     xi_Hi_traj(1,1,:) =xi_Fi_traj(1,swing,:) ;
    %                     xi_Hi_traj(1,2,:) =xi_Fi_traj(1,swing,:) ;
    
end
global control timings cc timeDT Test
control= [ones(1,3),ones(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
     Test=1;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    separateChains=1;
    count=1; cc=1; timeDT=0;
   timings=0;
 
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    
%
    control= [zeros(1,3),zeros(1,3),ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),zeros(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
Test=2;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    separateChains=1;
    count=1; cc=1; timeDT=0;
   timings=0;
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
  %%  
control= [ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
Test=3;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    separateChains=1;
    count=1; cc=1; timeDT=0;
   timings=0;
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
  %%  
control= [ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
   Test=4;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    separateChains=1;
    count=1; cc=1; timeDT=0;
   timings=0;
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    
control= [ones(1,3),[1,1,0],ones(1,3),[1,1,0],ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
   Test=5;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    separateChains=1;
    count=1; cc=1; timeDT=0;
   timings=0;
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
    

    
control= [ones(1,3),[1,1,0],ones(1,3),[1,1,1],ones(1,3),zeros(1,3),ones(1,3),zeros(1,3),];
   count=1; cc=1; timeDT=0;
   timings=0;
   Test=6;
    separateChains=0;
     
    [theta_arm,theta_leg] = TestJacob (t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,theta_head,swing);
 