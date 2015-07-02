function [ xi_B ] = resolved_momentum_control( P, L, xi_Fi, xi_Hi)
%RESOLVED_MOMENTUM_CONTROL calculate the velocity of the waist using the
%resolved momentum control formulas
%   PL : momentum and angular momentum of the robot PL = [P,L]'
%   eps_Fi : velocity of the feet eps_Fi = [v_Fi, w_Fi]'
%   eps_Hi : velocity of the hands eps_Fi = [v_Fi, w_Fi]'

%   eps_B : velocity of the waist eps_B = [v_B,w_B]

%% Global variables
global Mt_leg_const;
global Ht_leg_const;
global Mt_arm_const;
global Ht_arm_const;
global Mt_waist_const;
global Ht_waist_const;

%% Initialisation
inertia_leg_constrained = [Mt_leg_const; Ht_leg_const];
inertia_arm_constrained = [Mt_arm_const; Ht_arm_const];
inertia_waist_constrained = [Mt_waist_const; Ht_waist_const];

%% Calculation of y
temp_leg = zeros(6,1);
temp_arm = zeros(6,1);
for i=1:2
    % the inertia matrices are multiplied with the reference velocities
    temp_leg = temp_leg + inertia_leg_constrained(:,1+(i-1)*6:i*6)*xi_Fi(:,i);
    temp_arm = temp_arm + inertia_arm_constrained(:,1+(i-1)*6:i*6)*xi_Hi(:,i);
end
y = [P;L] - temp_leg - temp_arm;

%% Calculation of the velocity
 xi_B = inertia_waist_constrained\y;

end

