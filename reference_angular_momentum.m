function [ dtheta_leg, dtheta_arm ] = reference_angular_momentum( xi_B, xi_Fi, xi_Hi )
%REFERENCE_ANGULAR_MOMENTUM calculate the joint velocities for the arm and
%the legs
%   eps_B : velocities of the waist
%   eps_Fi : velocities of the feet
%   eps_Hi : velocities of the hands

%   dtheta_leg : joint velocities of the legs
%   dtheta_arm : joint velocities of the arms

%% Declaration of the global variables
global J_leg_inv;
global J_arm_inv;
global r_B_Fi;
global r_B_Hi;

%% Calculation of the legs joint velocities
% calculation of the temporary matrices to simplify the calcul
mat_leg_1 = [eye(3,3), -skew(r_B_Fi(:,1)); zeros(3,3), eye(3,3)];
mat_leg_2 = [eye(3,3), -skew(r_B_Fi(:,2)); zeros(3,3), eye(3,3)];

% calculation of the results for leg 1
temp_leg_1 = J_leg_inv(:,1:6)*xi_Fi(:,1) - J_leg_inv(:,1:6)*mat_leg_1*xi_B;
% calculation of the results for leg 2
temp_leg_2 = J_leg_inv(:,7:12)*xi_Fi(:,2) - J_leg_inv(:,7:12)*mat_leg_2*xi_B;

% concatenation of the results
dtheta_leg = [temp_leg_1, temp_leg_2];

%% Calculation of the arms joint velocities
% calculation of the temporary matrices to simplify the calcul
mat_arm_1 = [eye(3,3), -skew(r_B_Hi(:,1)); zeros(3,3), eye(3,3)];
mat_arm_2 = [eye(3,3), -skew(r_B_Hi(:,2)); zeros(3,3), eye(3,3)];

% calculation of the results for arm 1
temp_arm_1 = J_arm_inv(:,1:6)*xi_Hi(:,1) - J_arm_inv(:,1:6)*mat_arm_1*xi_B;
% calculation of the results for leg 2
temp_arm_2 = J_arm_inv(:,7:12)*xi_Hi(:,2) - J_arm_inv(:,7:12)*mat_arm_2*xi_B;

% concatenation of the results
dtheta_arm = [temp_arm_1, temp_arm_2];
end

