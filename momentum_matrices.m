function [ Mt, Ht, It ] = momentum_matrices( chain_leg_R, chain_leg_L, chain_arm_R,...
    chain_arm_L, chain_head )
%calculate_momentum_matrices calculate the matrices of inertia M and H for the robot
%   chain_leg_R : kinematic chain of the right leg
%   chain_leg_L : kinematic chain of the left leg
%   chain_arm_R : kinematic chain of the right arm
%   chain_arm_L : kinematic chain of the left arm

%   Mt : 
%   Ht :

%% Global variables
global size_arms;
global size_legs;
global size_head;

global I_arm_R;
global M_arm_R;
global com_arm_R

global I_arm_L;
global M_arm_L;
global com_arm_L;

global I_leg_R;
global M_leg_R;
global com_leg_R

global I_leg_L;
global M_leg_L;
global com_leg_L;

global I_head;
global M_head;
global com_head;

global I_torso;
global M_torso;
global com_torso;

global P_COG;

global Mt_leg;
global Ht_leg
global Mt_arm;
global Ht_arm;

global T_global;

%% Matrix of the right leg
T_leg1 = chain_leg_R(:,:,1:end-1);
% Calculation of the total masses and inertia matrices of the entire body
 [M_leg1,H_leg1,I_l1,mh_leg1,ch_leg1 ] = calculate_inertia_matrix( M_leg_R, com_leg_R, I_leg_R, T_leg1(1:3,1:3,:),T_leg1(1:3,4,:) );

 
%% Matrix of the left leg
T_leg2 = chain_leg_L(:,:,1:end-1);
% Calculation of the total masses and inertia matrices of the entire body
[M_leg2,H_leg2,I_l2,mh_leg2,ch_leg2  ]= calculate_inertia_matrix( M_leg_L, com_leg_L, I_leg_L, T_leg2(1:3,1:3,:),T_leg2(1:3,4,:) );
 

%% Matrix of the right arm
T_arm1 = chain_arm_R(:,:,1:end-1);
% Calculation of the total masses and inertia matrices of the entire body
[M_arm1,H_arm1,I_a1,mh_arm1,ch_arm1 ] = calculate_inertia_matrix( M_arm_R, com_arm_R, I_arm_R, T_arm1(1:3,1:3,:),T_arm1(1:3,4,:) );



%% Matrix of the left arm
T_arm2 = chain_arm_L(:,:,1:end-1);
% Calculation of the total masses and inertia matrices of the entire body
[M_arm2,H_arm2,I_a2,mh_arm2,ch_arm2 ] = calculate_inertia_matrix( M_arm_L, com_arm_L, I_arm_L, T_arm2(1:3,1:3,:),T_arm2(1:3,4,:) );


%% Matrix of the head
T_head = chain_head(:,:,1:2);
% Calculation of the total masses and inertia matrices of the entire body
[M_h,H_h,I_h,mh_h,ch_h ] = calculate_inertia_matrix( M_head, com_head, I_head, T_head(1:3,1:3,:),T_head(1:3,4,:) );

%% Calculation of the base of the tree structure
m_hat = M_torso + mh_leg1 + mh_leg2 + mh_arm1 + mh_arm2 + mh_h; %to consider the head
c_hat = (mh_leg1*ch_leg1 + mh_leg2*ch_leg2 + mh_arm1*ch_arm1 + mh_arm2*ch_arm2 + mh_h*ch_h ...%to consider the head
    + M_torso*(T_global(1:3,1:3)*com_torso+T_global(1:3,4))) / m_hat;
    I_hat = I_l1(:,:,1) + mh_leg1*D(ch_leg1-c_hat) + ...
            I_l2(:,:,1) + mh_leg2*D(ch_leg2-c_hat) + ...
            I_a1(:,:,1) + mh_arm1*D(ch_arm1-c_hat) + ...
            I_a2(:,:,1) + mh_arm2*D(ch_arm2-c_hat) + ...%
            I_h(:,:,1) + mh_h*D(ch_h-c_hat) + ...
            T_global(1:3,1:3)*I_torso*T_global(1:3,1:3)'+...
            M_torso*D((T_global(1:3,1:3)*com_torso+T_global(1:3,4))-c_hat);


% calculate H using M and H0 
H_leg1 = H_leg1 - skew(c_hat)*M_leg1; 
H_leg2 = H_leg2 - skew(c_hat)*M_leg2;
H_arm1 = H_arm1 - skew(c_hat)*M_arm1; 
H_arm2 = H_arm2 - skew(c_hat)*M_arm2; 
%NOTE: c_hat should be approximately P_B_COM the center of mass of robot
%if also the head is used to calculate m_hat, c_hat -> chat= P_B_COM
% could also be used to get center of mass and not use function center of
% gravity
% H = H0 - skew(P_B_COG)*M; %to the COG of the robot
%P_B_COG=c_hat;
%c_hat
%% Extract the inertia matrices for all the segments
Mt_leg = [M_leg1 , M_leg2 ];
Ht_leg = [H_leg1 , H_leg2 ];
Mt_arm = [M_arm1 , M_arm2 ];
Ht_arm = [H_arm1 , H_arm2 ];

%% Calculation of the matrices M and H
Mt = [Mt_leg,Mt_arm];
Ht = [Ht_leg,Ht_arm];
It=I_hat;
end


