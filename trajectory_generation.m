function [P_F,p_k,xi_Fi_traj,lambda_traj, n_k,swing]= trajectory_generation(nb_steps,nb_points,step_length,step_time,T_leg_R,T_leg_L,max_height,mid_height)
%% Trajectory generation
% Inputs:
%   nb_steps: number of steps to be generated (each movement of a foot is
%    considered a step)
%   nb_points: number by which each step will be discretized
%   step_length: the length of a step
%   step_time: time duration of a single step
%   T_leg_R: initial position of the right foot
%   T_leg_L: initial position of the left foot
%   max_height: maximun height in to which the feet will rise
%   mid_height:
% Outputs: 
%   P_F:Position of the feet
%    P_SR_traj: position of the contact of the right foot
%    P_SL_traj: position of the contact of the left foot
%   xi_Fi_traj: Twist of the feet
%    lambda_traj:array of coefficients of force at every contact point ...
%           (in this case the only contacts considered are the feet)...
%           ,this will create an even force distribution
% Other required info:
%   lambda_coeff: general coefficient for lambda that will create an even distribution of forces 
%   
start=1;
swing_foot=1;
xi_Fi_traj=zeros(6,2,1);
P_F(:,1,1) = T_leg_R(1:3,4);
P_F(:,2,1) = T_leg_L(1:3,4);
%P_SR_traj=P_SR_trajs;
%P_SL_traj=P_SL_trajs;
swing(1)=swing_foot;
static=0;
for s = 1:nb_steps
    
    % generate the trajectory for one step
    if start == 1
        if static==0
            %         [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
            %             generate_stepNoBck(step_length/2,0,step_time,nb_points,...
            %             swing_foot,T_leg_R,T_leg_L,max_height,mid_height);
            
            [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
                generateStep(step_length/2,0,step_time,nb_points,...
                swing_foot,T_leg_R,T_leg_L,max_height,mid_height);
        else
            [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
                generate_static_step(step_length,0,step_time,nb_points,...
                swing_foot);
            
        end
        start = 0;
    else
        
        T_leg_R = makehgtform('translate',P_F(:,1,end));
        T_leg_L = makehgtform('translate',P_F(:,2,end));
        
        if static ==0
            %         [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
            %             generate_stepNoBck(step_length,0,step_time,nb_points,...
            %             swing_foot,T_leg_R,T_leg_L,max_height,mid_height);
            [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
                generateStep(step_length,0,step_time,nb_points,...
                swing_foot,T_leg_R,T_leg_L,max_height,mid_height);
        else
            [P_Fs, P_SR_trajs, P_SL_trajs, xi_Fi_trajs, lambda_trajs] = ...
                generate_static_step(step_length,0,step_time,nb_points,...
                swing_foot);
            
            
        end
    end
    
    %P_F(:,:,(s-1)*nb_points+1:(s-1)*nb_points+nb_points+1)=P_Fs;
    P_F=cat(3,P_F,P_Fs);
    P_SR_traj(:,:,(s-1)*nb_points+2:(s-1)*nb_points+nb_points+1)=P_SR_trajs;
    P_SL_traj(:,:,(s-1)*nb_points+2:(s-1)*nb_points+nb_points+1)=P_SL_trajs;
    xi_Fi_traj(:,:,(s-1)*nb_points+2:(s-1)*nb_points+nb_points+1)=xi_Fi_trajs;
    lambda_traj(:,(s-1)*nb_points+2:(s-1)*nb_points+nb_points+1)=lambda_trajs;
    swing((s-1)*nb_points+1:(s-1)*nb_points+nb_points)=ones(nb_points,1)*swing_foot;
    
    disp('generating trajectory of step : ');disp(s);
    % change the swing foot, note the change of swing foot takes place
    % at the end of the step or when both feet are at ground level ;
    if swing_foot == 1
        swing_foot = 2;
    else
        swing_foot = 1;
    end
    
    
end

swing((s-1)*nb_points+nb_points+1)=swing_foot; %extra value added for size matching
P_SR_traj(:,:,1)=P_SR_traj(:,:,2);
P_SL_traj(:,:,1)=P_SL_traj(:,:,2);
lambda_traj(:,1)=lambda_traj(:,2);
n_k=[zeros(1,size(lambda_traj,1));zeros(1,size(lambda_traj,1));ones(1,size(lambda_traj,1))];
    p_k=[P_SR_traj,P_SL_traj];



