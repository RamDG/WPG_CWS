function [ ] = inertia_constrained( )
%INERTIA_CONSTRAINED calculate the inertia matrices under constraint
%   segment : part of the body to consider (leg, arm or waist)

%% declaration of the global variables
global total_mass;
global total_inertia;
global size_arms;
global size_legs;

global Mt_leg;
global Ht_leg
global Mt_arm;
global Ht_arm;
global Mt_leg_const;
global Ht_leg_const;
global Mt_arm_const;
global Ht_arm_const;
global Mt_waist_const;
global Ht_waist_const;

global J_leg_inv;
global J_arm_inv;
global r_B_Fi;
global r_B_Hi;
global r_B_COG;

%% Calculation of the inertia matrices under constraint of the legs
% concatenate the inertia matrices
inertia_leg = [Mt_leg; Ht_leg];
% for both legs multiply the inertia matrices with the jacobian inverse
leg1 = inertia_leg(:,1:size_legs)*J_leg_inv(:,1:6);
leg2 = inertia_leg(:,size_legs+1:end)*J_leg_inv(:,7:12);
% concatenate both legs
res_legs = [leg1, leg2];
Mt_leg_const = res_legs(1:3,:);
Ht_leg_const = res_legs(4:6,:);

%% Calculation of the inertia matrices under constraint of the arms

% concatenate the inertia matrices
inertia_arm = [Mt_arm; Ht_arm];
% for both arms multiply the inertia matrices with the jacobian inverse
arm1 = inertia_arm(:,1:size_arms)*J_arm_inv(:,1:6);
arm2 = inertia_arm(:,size_arms+1:end)*J_arm_inv(:,7:12);
% concatenate both legs
res_arms = [arm1, arm2];
Mt_arm_const = res_arms(1:3,:);
Ht_arm_const = res_arms(4:6,:);

%% Calculation of the inertia matrices under constraint of the waist

% use the formula developped in Hirukawa2007
temp_waist = [total_mass*eye(3,3), -total_mass*skew(r_B_COG); zeros(3,3), total_inertia];
res_leg = zeros(6,6);
res_arm = zeros(6,6);
inertia_leg_constrained = [Mt_leg_const; Ht_leg_const];
inertia_arm_constrained = [Mt_arm_const; Ht_arm_const];

% sum over the two legs and tow arms
for i=1:2
    % create temporary matrix and result for the legs
    temp_leg = [eye(3,3), -skew(r_B_Fi(:,i)); zeros(3,3), eye(3,3)];
    res_leg = res_leg + inertia_leg_constrained(:,1+(i-1)*6:i*6)*temp_leg;
    % FIXME: skew(P_B_Fi(:,i)) could need to be -skew(P_B_Fi(:,i))
    % create temporary matrix and result for the arms
    temp_arm = [eye(3,3), -skew(r_B_Hi(:,i)); zeros(3,3), eye(3,3)];
    res_arm = res_arm + inertia_arm_constrained(:,1+(i-1)*6:i*6)*temp_arm;
end
% add the results of the legs and arms
%res_waist = temp_waist - 2*res_leg - 2*res_arm;%FIXME: why multiply by 2?
res_waist = temp_waist - res_leg - res_arm;
Mt_waist_const = res_waist(1:3,:);
Ht_waist_const = res_waist(4:6,:);

end


