 dp_Fi(:,1)=integrate(xi_Fi(1:3,1),chain_leg_R(1:3,4,end));
         dp_Fi(:,2)=integrate(xi_Fi(1:3,2),chain_leg_L(1:3,4,end));
         dp_Hi(:,1)=integrate(xi_Hi(1:3,1),chain_arm_R(1:3,4,end));
          dp_Hi(:,2)=integrate(xi_Hi(1:3,2),chain_arm_L(1:3,4,end));
          dp_B(:)=integrate(xi_B_ref(1:3,1),T_global(1:3,4));
          
           p_Fi(:,1)=chain_leg_R(1:3,4,end);
        p_Fi(:,2)=chain_leg_L(1:3,4,end);
        p_Hi(:,1)=chain_arm_R(1:3,4,end);
        p_Hi(:,2)=chain_arm_L(1:3,4,end);
        p_B=T_global(1:3,4);
          it=0;
          error=ones(3,1);
          error=p_Fi(:,2)-dp_Fi(:,2);
             t_arm_1 =t_arm_0;
        t_leg_1 =t_leg_0;
        
            while (sum(abs(error))>.0001)
        %Calculate joint velocities from reference twists
        
%     %% Declaration of the global variables
global J_leg_inv;
global J_arm_inv;
global r_B_Fi;
global r_B_Hi;
global T_global
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
temp_leg_1 = J_leg_inv(:,1:6)*xi_Fi(:,1) - J_leg_inv(:,1:6)*mat_leg_1*xi_B_ref;
% calculation of the results for leg 2
temp_leg_2 = J_leg_inv(:,7:12)*xi_Fi(:,2) - J_leg_inv(:,7:12)*mat_leg_2*xi_B_ref;

% concatenation of the results
dtheta_leg = [temp_leg_1, temp_leg_2];

%% Calculation of the arms joint velocities
% calculation of the temporary matrices to simplify the calcul
mat_arm_1 = [eye(3,3), -skew(r_B_Hi(:,1)); zeros(3,3), eye(3,3)];
mat_arm_2 = [eye(3,3), -skew(r_B_Hi(:,2)); zeros(3,3), eye(3,3)];

% calculation of the results for arm 1
temp_arm_1 = J_arm_inv(:,1:6)*xi_Hi(:,1) - J_arm_inv(:,1:6)*mat_arm_1*xi_B_ref;
% calculation of the results for leg 2
temp_arm_2 = J_arm_inv(:,7:12)*xi_Hi(:,2) - J_arm_inv(:,7:12)*mat_arm_2*xi_B_ref;

% concatenation of the results
dtheta_arm = [temp_arm_1, temp_arm_2];

%         global invJac
%         dtet=invJac*error;
%         dt_leg=[dtet(6:-1:1),dtet(6:11)];
%         dt_arm=[dtet(12:15),dtet(17:20)];
        dt_leg= dtheta_leg;
        dt_arm=dtheta_arm;

        % integrate the joint velocities to get the angles values
        theta_arm(:,:,s) = dt_arm+ t_arm_1;
        theta_leg(:,:,s) = dt_leg+ t_leg_1;
        
        %ensure that the angular value is between [-pi/2;pi/2]
       theta_arm(:,:,s) = mod_interval(theta_arm(:,:,s),-2*pi,2*pi);
       theta_leg(:,:,s) = mod_interval(theta_leg(:,:,s),-2*pi,2*pi);
        
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),s);
        % animate the motion
        if animation == 1
            animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
       p_Fi(:,1)=chain_leg_R(1:3,4,end);
        p_Fi(:,2)=chain_leg_L(1:3,4,end);
        p_Hi(:,1)=chain_arm_R(1:3,4,end);
        p_Hi(:,2)=chain_arm_L(1:3,4,end);
        p_B=T_global(1:3,4);
        
        error=dp_Fi(:,2)-p_Fi(:,2);
        it=it+1;
         disp ( error )
         disp (it)
          t_arm_1 = theta_arm(:,:,s);
        t_leg_1 = theta_leg(:,:,s);
            end
            disp ( strcat('converged with ',num2str(it),'iterations' ))
            disp (error)
            
           theta_arm(:,:,s)-  t_arm_0 
         theta_leg(:,:,s)-t_leg_0 
        
        
        
        