function [ T_end, T_chain ] = forward_chain( chain, theta_ref )
%FORWARD_CHAIN compute the forward kinematic of a kinematic chain with the
%angle value as input
%   chain : the kinematic chain to compute (leg_R/leg_L/arm_R/arm_L/head)
%   theta : the values of the joint angles

%   T_end : transformation matrix of the end effector wrt the base
%   T_chain : successive transformations matrices for the angles

%% Global variables
global alpha_leg_R;
global d_leg_R;
global theta_leg_R;
global t_offset_leg_R;
global r_leg_R;
global t_base_leg_R;
global t_end_leg_R;
global rot_leg_R;
global rot_fix_leg_R;

global alpha_leg_L;
global d_leg_L;
global theta_leg_L;
global t_offset_leg_L;
global r_leg_L;
global t_base_leg_L;
global t_end_leg_L;
global rot_leg_L;
global rot_fix_leg_L;

global alpha_arm_R;
global d_arm_R;
global theta_arm_R;
global t_offset_arm_R;
global r_arm_R;
global t_base_arm_R;
global t_end_arm_R;
global rot_arm_R;
global rot_fix_arm_R;

global alpha_arm_L;
global d_arm_L;
global theta_arm_L;
global t_offset_arm_L;
global r_arm_L;
global t_base_arm_L;
global t_end_arm_L;
global rot_arm_L;
global rot_fix_arm_L;

global alpha_head;
global d_head;
global theta_head_coeff;
global t_offset_head;
global r_head;
global t_base_head;
global t_end_head;
global rot_head;
global rot_fix_head;

%% According to the chain in input we calculate the transformation matrices
if strcmp(chain,'head')
    % calculate the value of theta considering the offset
    theta = theta_head_coeff.*theta_ref + t_offset_head;
    % call the forward kinematic function with the right DH parameters
    [T_end,T_chain] = forward_kinematic(alpha_head, d_head, theta, r_head,...
        t_base_head, t_end_head, rot_head, rot_fix_head);
    
elseif strcmp(chain,'leg_R')
    % calculate the value of theta considering the offset
    theta = theta_leg_R.*theta_ref + t_offset_leg_R;
    % call the forward kinematic function with the right DH parameters
    [T_end,T_chain] = forward_kinematic(alpha_leg_R, d_leg_R, theta, r_leg_R,...
        t_base_leg_R, t_end_leg_R, rot_leg_R, rot_fix_leg_R);
    
elseif strcmp(chain,'leg_L')
    % calculate the value of theta considering the offset
    theta = theta_leg_L.*theta_ref + t_offset_leg_L;
    % call the forward kinematic function with the right DH parameters
    [T_end,T_chain] = forward_kinematic(alpha_leg_L, d_leg_L, theta, r_leg_L,...
        t_base_leg_L, t_end_leg_L, rot_leg_L, rot_fix_leg_L);
    
elseif strcmp(chain,'arm_R')
    % calculate the value of theta considering the offset
    theta = theta_arm_R.*theta_ref + t_offset_arm_R;
    % call the forward kinematic function with the right DH parameters
    [T_end,T_chain] = forward_kinematic(alpha_arm_R, d_arm_R, theta, r_arm_R,...
        t_base_arm_R, t_end_arm_R, rot_arm_R, rot_fix_arm_R);
    
elseif strcmp(chain,'arm_L')
    % calculate the value of theta considering the offset
    theta = theta_arm_L.*theta_ref + t_offset_arm_L;
    % call the forward kinematic function with the right DH parameters
    [T_end,T_chain] = forward_kinematic(alpha_arm_L, d_arm_L, theta, r_arm_L,...
        t_base_arm_L, t_end_arm_L, rot_arm_L, rot_fix_arm_L);
    
else
    T_end = zeros(4,4);
    T_chain = zeros(4,4);
    disp('Incorrect input');
end

