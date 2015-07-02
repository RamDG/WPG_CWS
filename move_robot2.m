function [ chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head ] = ...
    move_robot2( theta_leg, theta_arm, theta_head,swing_foot,nextSwing_foot,s )
%MOVE_ROBOT move the robot with the desired joint values
%   theta_leg : joint values for the legs ([theta_leg_R, theta_leg_L])
%   theta_arm : joint values for the arms ([theta_arm_R, theta_arm_L])

%   J_leg : Jacobian matrices of the legs
%   J_arm : Jacobian matrices of the arms
%   P_Fi : positions of the feet
%   P_Hi : positions of the hands

%% Global variables
global P_COG;
global P_Fi;
global P_Hi;
global Mt;
global Ht;
global J_arm_inv;
global J_leg_inv;
global total_inertia;
global r_B_Fi;
global r_B_Hi;
global r_B_COG;
global T_global;

global timings count
%testMDH;
TreeStructure;
%% Calculation of the COM

    P_COG = center_of_gravity( chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head );
   % P_COG
%% Position of the feet and hands
P_Fi = [chain_leg_R(1:3,4,end),chain_leg_L(1:3,4,end)];
P_Hi = [chain_arm_R(1:3,4,end),chain_arm_L(1:3,4,end)];

%% position of legs and arms from the origin of the waist to ith foot coordinates wrt to global reference
r_B_Fi = [chain_leg_R(1:3,4,end)-T_global(1:3,4),chain_leg_L(1:3,4,end)-T_global(1:3,4)];
r_B_Hi = [chain_arm_R(1:3,4,end)-T_global(1:3,4),chain_arm_L(1:3,4,end)-T_global(1:3,4)];
r_B_COG = P_COG-T_global(1:3,4);


%% Jacobian matrices
global Jac invJac
if swing_foot==1
   [J_RFoot,J_Torso,J_RHand,J_LHand]= LeftJacobians(q');
   Jac=[J_RFoot;J_Torso;J_RHand;J_LHand];
else
   [J_LFoot,J_Torso,J_RHand,J_LHand]= RightJacobians(q');
   Jac=[J_LFoot;J_Torso;J_RHand;J_LHand];
end
invtime=tic;
invJac=inv_jacobian(Jac(:,1:21));
timings(count)=toc(invtime);
count=count+1;
%disp('inverse Jacobian calcualtion time')
%% Inertia matrices
[Mt,Ht,total_inertia] = momentum_matrices(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);

% matrices under constraint
%inertia_constrained(); TODO: Changes required to use the full jacobian on
%the inertia 

end

