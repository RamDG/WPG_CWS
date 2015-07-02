function [ cog_pos ] = center_of_gravity( chain_leg_R, chain_leg_L, chain_arm_R,...
    chain_arm_L, chain_head )
%CENTER_OF_GRAVITY calculate the position of the COG of the robot wrt the
%torso frame
%   chain_leg_R : kinematic chain of the right leg
%   chain_leg_L : kinematic chain of the left leg
%   chain_arm_R : kinematic chain of the right arm
%   chain_arm_L : kinematic chain of the left arm
%   chain_head : kinematic chain of the head

%   cog_pos : relative position of the COG of the robot

%% Global variables
global M_leg_R;
global M_leg_L;
global M_arm_R;
global M_arm_L;
global M_head;
global M_torso;
global total_mass;

global com_leg_R;
global com_leg_L;
global com_arm_R;
global com_arm_L;
global com_head;
global com_torso;

global T_global;

%% For each kinematic chains calculate the weighted matrices
% initialisation of the weighted matrix
weighted_pos = zeros(3,1);

% calculation for the both legs
for i = 1:length(M_leg_R)
    % multiply the transformation matrix with the translation to the local
    % com
    end_R = makehgtform('translate',com_leg_R(:,i)');
    T_R = chain_leg_R(:,:,i)*end_R;
    end_L = makehgtform('translate',com_leg_L(:,i)');
    T_L = chain_leg_L(:,:,i)*end_L;
    
    % extract the position of the joint from the transformation matrix
    pos_i_R = T_R(1:3,4);
    pos_i_L = T_L(1:3,4);
    % multiply it with the mass of the segment and add to the weight matrix
    weighted_pos = weighted_pos + pos_i_R*M_leg_R(i) + pos_i_L*M_leg_L(i);
     
end

% calculation for the both arms
for i = 1:length(M_arm_R)
    % multiply the transformation matrix with the translation to the local
    % com
    end_R = makehgtform('translate',com_arm_R(:,i)');
    T_R = chain_arm_R(:,:,i)*end_R;
    end_L = makehgtform('translate',com_arm_L(:,i)');
    T_L = chain_arm_L(:,:,i)*end_L;
    
    % extract the position of the joint from the transformation matrix
    pos_i_R = T_R(1:3,4);
    pos_i_L = T_L(1:3,4);
    % multiply it with the mass of the segment and add to the weight matrix
    weighted_pos = weighted_pos + pos_i_R*M_arm_R(i) + pos_i_L*M_arm_L(i);
    
end

% calculation for the head
for i = 1:length(M_head)
    % multiply the transformation matrix with the translation to the local
    % com
    end_head= makehgtform('translate',com_head(:,i)');
    T_head = chain_head(:,:,i)*end_head;
    
    % extract the position of the joint from the transformation matrix
    pos_i = T_head(1:3,4);
    % multiply it with the mass of the segment and add to the weight matrix
    weighted_pos = weighted_pos + pos_i*M_head(i);
end

% calculation for the torso, the transformation matrix is made only by the
% translation to the COM of the torso
T_torso =T_global* makehgtform('translate',com_torso(:,1)');
% extract the position of the joint from the transformation matrix
pos_torso = T_torso(1:3,4);
% multiply it with the mass of the segment and add to the weight matrix
weighted_pos = weighted_pos + pos_torso*M_torso;

%% Calculation of the COG position
% finally simply divide the weighted matrix by the total mass
cog_pos = weighted_pos/total_mass;

end
