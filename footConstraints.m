function [xi_Fi]=footConstraints(dtheta_leg,xi_B)
%% Matrix for checking the feet velocites with the feet constraints
global r_B_Fi;
global r_B_Hi;
global J_leg_R J_leg_L J_arm_R J_arm_L
%% Calculation of the legs joint velocities
% calculation of the temporary matrices to simplify the calcul
mat_leg_1 = [eye(3,3), -skew(r_B_Fi(:,1)); zeros(3,3), eye(3,3)];
mat_leg_2 = [eye(3,3), -skew(r_B_Fi(:,2)); zeros(3,3), eye(3,3)];

xi_Fi(:,1)=mat_leg_1*xi_B+ J_leg_R*dtheta_leg(:,1);
xi_Fi(:,2)=mat_leg_2*xi_B+ J_leg_L*dtheta_leg(:,2);