function [ dtheta_leg, dtheta_arm,theta_leg, theta_arm ] = foot_constraints2( xi_B, xi_Fi, xi_Hi,chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s )
 %Calculate joint velocities from reference twists
        
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
global T_global
global animation;

% Position
dp_Fi(:,1)=integrate(xi_Fi(1:3,1),chain_leg_R(1:3,4,end));
dp_Fi(:,2)=integrate(xi_Fi(1:3,2),chain_leg_L(1:3,4,end));
dp_Hi(:,1)=integrate(xi_Hi(1:3,1),chain_arm_R(1:3,4,end));
dp_Hi(:,2)=integrate(xi_Hi(1:3,2),chain_arm_L(1:3,4,end));
dp_B=integrate(xi_B(1:3,1),T_global(1:3,4));

p_Fi(:,1)=chain_leg_R(1:3,4,end);
p_Fi(:,2)=chain_leg_L(1:3,4,end);
p_Hi(:,1)=chain_arm_R(1:3,4,end);
p_Hi(:,2)=chain_arm_L(1:3,4,end);
p_B=T_global(1:3,4);


errorFi(:,1)=dp_Fi(:,1)-p_Fi(:,1);
errorFi(:,2)=dp_Fi(:,2)-p_Fi(:,2);
errorHi(:,1)=dp_Hi(:,1)-p_Hi(:,1);
errorHi(:,2)=dp_Hi(:,2)-p_Hi(:,2);
errorB=dp_B-p_B;

%Orientation
% do_Fi(:,1)=integrate(xi_Fi(1:3,1),chain_leg_R(1:3,4,end));
% do_Fi(:,2)=integrate(xi_Fi(1:3,2),chain_leg_L(1:3,4,end));
% do_Hi(:,1)=integrate(xi_Hi(1:3,1),chain_arm_R(1:3,4,end));
% do_Hi(:,2)=integrate(xi_Hi(1:3,2),chain_arm_L(1:3,4,end));
% do_B=integrate(xi_B(1:3,1),T_global(1:3,4));
% 
% o_Fi(:,1)=chain_leg_R(1:3,4,end);
% o_Fi(:,2)=chain_leg_L(1:3,4,end);
% o_Hi(:,1)=chain_arm_R(1:3,4,end);
% o_Hi(:,2)=chain_arm_L(1:3,4,end);
% o_B=T_global(1:3,4);
% 
% 
% errorFi(:,1)=do_Fi(:,1)-o_Fi(:,1);
% errorFi(:,2)=do_Fi(:,2)-o_Fi(:,2);
% errorHi(:,1)=do_Hi(:,1)-o_Hi(:,1);
% errorHi(:,2)=do_Hi(:,2)-o_Hi(:,2);
% errorB=do_B-o_B;

error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
t_arm_1 =t_arm_0;
t_leg_1 =t_leg_0;
theta_arm(:,:) = t_arm_1;
theta_leg(:,:) =t_leg_1;
it=0;
while (sum(abs(error))>.0001 && it<500)
    
    %% Calculation of the legs joint velocities
    %test to align the waist with the first axis of the feet since its
    %different
    % R_B_1=chain_leg_R(1:3,1:3,end)'*T_global(1:3,1:3);
    % L_B_1=chain_leg_L(1:3,1:3,end)'*T_global(1:3,1:3);
    % R_1_B=T_global(1:3,1:3)'*chain_leg_R(1:3,1:3,1);
    % L_1_B=T_global(1:3,1:3)'*chain_leg_L(1:3,1:3,1);
    %
    % mat_l_1 = [R_1_B, -skew(r_B_Fi(:,1))*R_1_B; zeros(3,3), R_1_B];
    % mat_l_2 = [L_1_B, -skew(r_B_Fi(:,2))*L_1_B; zeros(3,3), L_1_B];
    
    % calculation of the temporary matrices to simplify the calcul
    mat_leg_1 = [eye(3,3), -skew(r_B_Fi(:,1)); zeros(3,3), eye(3,3)];
    mat_leg_2 = [eye(3,3), -skew(r_B_Fi(:,2)); zeros(3,3), eye(3,3)];
    % mat_leg_1 = [R_B_1, -skew(r_B_Fi(:,1))*R_B_1; zeros(3,3), R_B_1];
    % mat_leg_2 = [L_B_1, -skew(r_B_Fi(:,2))*L_B_1; zeros(3,3), L_B_1];
    
    % calculation of the results for leg 1
    %temp_leg_1 = J_leg_inv(:,1:6)*xi_Fi(:,1) - J_leg_inv(:,1:6)*mat_leg_1*xi_B;
    temp_leg_1 = J_leg_inv(:,1:6)*[errorFi(:,1);zeros(3,1)] - J_leg_inv(:,1:6)*mat_leg_1*[errorB;zeros(3,1)];
    
    % calculation of the results for leg 2
    %temp_leg_2 = J_leg_inv(:,7:12)*xi_Fi(:,2) - J_leg_inv(:,7:12)*mat_leg_2*xi_B;
    temp_leg_2 = J_leg_inv(:,7:12)*[errorFi(:,2);zeros(3,1)] - J_leg_inv(:,7:12)*mat_leg_2*[errorB;zeros(3,1)];
    
    % concatenation of the results
    dtheta_leg = [temp_leg_1, temp_leg_2];
    
    %% Calculation of the arms joint velocities
    % calculation of the temporary matrices to simplify the calcul
    mat_arm_1 = [eye(3,3), -skew(r_B_Hi(:,1)); zeros(3,3), eye(3,3)];
    mat_arm_2 = [eye(3,3), -skew(r_B_Hi(:,2)); zeros(3,3), eye(3,3)];
    
    % calculation of the results for arm 1
    %temp_arm_1 = J_arm_inv(:,1:6)*xi_Hi(:,1) - J_arm_inv(:,1:6)*mat_arm_1*xi_B;
    temp_arm_1 = J_arm_inv(:,1:6)*[errorHi(:,1);zeros(3,1)] - J_arm_inv(:,1:6)*mat_arm_1*[errorB;zeros(3,1)];
    % calculation of the results for leg 2
    %temp_arm_2 = J_arm_inv(:,7:12)*xi_Hi(:,2) - J_arm_inv(:,7:12)*mat_arm_2*xi_B;
    temp_arm_2 = J_arm_inv(:,7:12)*[errorHi(:,2);zeros(3,1)] - J_arm_inv(:,7:12)*mat_arm_2*[errorB;zeros(3,1)];
    
    % concatenation of the results
    dtheta_arm = [temp_arm_1, temp_arm_2];
    
    %%
    
    %         global invJac
    %         dtet=invJac*error;
    %         dt_leg=[dtet(6:-1:1),dtet(6:11)];
    %         dt_arm=[dtet(12:15),dtet(17:20)];
    dt_leg= dtheta_leg;
    dt_arm=dtheta_arm;
    
    % integrate the joint velocities to get the angles values
    theta_arm = dt_arm+ t_arm_1;
    theta_leg = dt_leg+ t_leg_1;
    
    %ensure that the angular value is between [-pi/2;pi/2]
    theta_arm = mod_interval(theta_arm,-2*pi,2*pi);
    theta_leg = mod_interval(theta_leg,-2*pi,2*pi);
    
  
    % move the robot with the new desired joint values, update
    % jacobians and intertia matrices
    [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
        move_robot(theta_leg,theta_arm,theta_head,swing(s),swing(s),s);
    %         % animate the motion
    %         if animation == 1
    %             animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
    %         end
    p_Fi(:,1)=chain_leg_R(1:3,4,end);
    p_Fi(:,2)=chain_leg_L(1:3,4,end);
    p_Hi(:,1)=chain_arm_R(1:3,4,end);
    p_Hi(:,2)=chain_arm_L(1:3,4,end);
    p_B=T_global(1:3,4);
    
    errorFi(:,1)=dp_Fi(:,1)-p_Fi(:,1);
    errorFi(:,2)=dp_Fi(:,2)-p_Fi(:,2);
    errorHi(:,1)=dp_Hi(:,1)-p_Hi(:,1);
    errorHi(:,2)=dp_Hi(:,2)-p_Hi(:,2);
    errorB=dp_B-p_B;
    error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
    it=it+1;
    %          disp ( error )
    %          disp (it)
    t_arm_1 = theta_arm;
    t_leg_1 = theta_leg;
end
%%
if error<.0001
    %              disp ( strcat('converged with ',num2str(it),' iterations', ' point ',num2str(s ), ' of trajectory'))
    %              disp (error)
else
    
    disp(strcat('no convergence after 500 iterations' , ' point ',num2str(s ), ' of trajectory'))
    errorFi
    errorHi
    errorB
end
%            theta_arm(:,:)-  t_arm_0
%          theta_leg(:,:)-t_leg_0

dtheta_leg=differentiate( t_leg_1,t_leg_0 );
dtheta_arm=differentiate( t_arm_1,t_arm_0 );