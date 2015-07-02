function [ ] = draw_trajectory( P_F, P_SR, P_SL, nb_points )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

global chain_leg_R;
global chain_leg_L;
global chain_arm_R;
global chain_arm_L;
global chain_head;

for i = 1:nb_points
    animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
    PR = zeros(3,nb_points);
    PL = zeros(3,nb_points);
    
    PR(:,:) = P_F(:,1,:);
    PL(:,:) = P_F(:,2,:);
    
    plot3(PR(1,:),PR(2,:),PR(3,:)); hold on;
    plot3(PL(1,:),PL(2,:),PL(3,:)); hold on;
    
    foot_R_X = zeros(1,5);
    foot_R_Y = zeros(1,5);
    foot_R_Z = zeros(1,5);
    
    foot_R_X(1,1:4) = P_SR(1,:,i);
    foot_R_Y(1,1:4) = P_SR(2,:,i);
    foot_R_Z(1,1:4) = P_SR(3,:,i);
    foot_R_X(5) = foot_R_X(1);
    foot_R_Y(5) = foot_R_Y(1);
    foot_R_Z(5) = foot_R_Z(1);
    plot3(foot_R_X, foot_R_Y, foot_R_Z); hold on;
    
    foot_L_X = zeros(1,5);
    foot_L_Y = zeros(1,5);
    foot_L_Z = zeros(1,5);
    
    foot_L_X(1,1:4) = P_SL(1,:,i);
    foot_L_Y(1,1:4) = P_SL(2,:,i);
    foot_L_Z(1,1:4) = P_SL(3,:,i);
    foot_L_X(5) = foot_L_X(1);
    foot_L_Y(5) = foot_L_Y(1);
    foot_L_Z(5) = foot_L_Z(1);
    plot3(foot_L_X, foot_L_Y, foot_L_Z); hold on;
    
    pause(0.02);
end
end

