function [ dtheta_leg, dtheta_arm,theta_leg, theta_arm,it ] = foot_constraints( xi_B, xi_Fi, xi_Hi,chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s )
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


%%
limits=0;
limit_vel=0;
animation=0;
% Dependency on matlab toolboxes
aerospaceTB=0;
roboticTB=1;
%% For limit checking
if limits==1
global q_lim_leg_R q_lim_leg_L q_lim_arm_R q_lim_arm_L %q_lim_head
global J_arm_R J_arm_L J_leg_R J_leg_L
end
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


errorFi(1:3,1)=dp_Fi(:,1)-p_Fi(:,1);
errorFi(1:3,2)=dp_Fi(:,2)-p_Fi(:,2);
errorHi(1:3,1)=dp_Hi(:,1)-p_Hi(:,1);
errorHi(1:3,2)=dp_Hi(:,2)-p_Hi(:,2);
errorB(1:3)=dp_B-p_B;

%Orientation

do_Fr=integrateTwistEulRotm(xi_Fi(4:6,1),chain_leg_R(1:3,1:3,end));
do_Fl=integrateTwistEulRotm(xi_Fi(4:6,2),chain_leg_L(1:3,1:3,end));
% do_Hi(:,1)=integrateTwistEulRotm(xi_Hi(4:6,1),chain_arm_R(1:3,1:3,end));
% do_Hi(:,2)=integrateTwistEulRotm(xi_Hi(4:6,2),chain_arm_L(1:3,1:3,end));
do_B=integrateTwistEulRotm(xi_B(4:6,1),T_global(1:3,1:3,end));


 if sum(sum(abs(chain_leg_R(1:3,1:3,end)-eye(3,3))))<.0000001
        chain_leg_R(1:3,1:3,end)=eye(3,3);
    end
    if sum(sum(abs(chain_leg_L(1:3,1:3,end)-eye(3,3))))<.0000001
        chain_leg_L(1:3,1:3,end)=eye(3,3);
    end
    o_Fr = rotm2quat(chain_leg_R(1:3,1:3,end));
    o_Fl = rotm2quat(chain_leg_L(1:3,1:3,end));
    % o_Hi(:,1) = rotm2quat(chain_arm_R(1:3,1:3,end));
    % o_Hi(:,2) = rotm2quat(chain_arm_L(1:3,1:3,end));
   o_B = rotm2quat(T_global(1:3,1:3,end));
    
    % %  use with robotics toolbox installed
    if roboticTB==1
        errorFi(6:-1:4,1)=quat2eul(quatmultiply(quatinv(o_Fr),do_Fr));
        errorFi(6:-1:4,2)=quat2eul(quatmultiply(quatinv(o_Fl),do_Fl));
        errorB(6:-1:4)=quat2eul(quatmultiply(quatinv(o_B),do_B));
    end
    % use with aerospace toolbox installed
    if aerospaceTB==1
        errorFi(6:-1:4,1)=quat2angle(quatmultiply(quatinv(o_Fr),do_Fr));
        errorFi(6:-1:4,2)=quat2angle(quatmultiply(quatinv(o_Fl),do_Fl));
        errorB(6:-1:4)=quat2angle(quatmultiply(quatinv(o_B),do_B));
    end
    errorFi(6,1)=0;%leave the error in the yaw direction free
    errorFi(6,2)=0;
    errorB(6)=0;
   
   
    errorHi(4:6,1)=zeros(3,1);
    errorHi(4:6,2)=zeros(3,1);
    %errorB(4:6)=zeros(3,1);
    errorB=errorB';
    
    error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
    refError=99;
    best_thArm=t_arm_0;
    best_thLeg=t_leg_0;
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
        temp_leg_1 = J_leg_inv(:,1:6)*errorFi(:,1) - J_leg_inv(:,1:6)*mat_leg_1*errorB;
        
        % calculation of the results for leg 2
        %temp_leg_2 = J_leg_inv(:,7:12)*xi_Fi(:,2) - J_leg_inv(:,7:12)*mat_leg_2*xi_B;
        temp_leg_2 = J_leg_inv(:,7:12)*errorFi(:,2) - J_leg_inv(:,7:12)*mat_leg_2*errorB;
        
        % concatenation of the results
        dtheta_leg = [temp_leg_1, temp_leg_2];
        
        %% Calculation of the arms joint velocities
        % calculation of the temporary matrices to simplify the calcul
        mat_arm_1 = [eye(3,3), -skew(r_B_Hi(:,1)); zeros(3,3), eye(3,3)];
        mat_arm_2 = [eye(3,3), -skew(r_B_Hi(:,2)); zeros(3,3), eye(3,3)];
        
        % calculation of the results for arm 1
        %temp_arm_1 = J_arm_inv(:,1:6)*xi_Hi(:,1) - J_arm_inv(:,1:6)*mat_arm_1*xi_B;
        temp_arm_1 = J_arm_inv(:,1:6)*errorHi(:,1) - J_arm_inv(:,1:6)*mat_arm_1*errorB;
        % calculation of the results for leg 2
        %temp_arm_2 = J_arm_inv(:,7:12)*xi_Hi(:,2) - J_arm_inv(:,7:12)*mat_arm_2*xi_B;
        temp_arm_2 = J_arm_inv(:,7:12)*errorHi(:,2) - J_arm_inv(:,7:12)*mat_arm_2*errorB;
        
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
        %% Check limits q_lim_leg_R q_lim_leg_L q_lim_arm_R q_lim_arm_L
        if limits==1
            [okR,breachedR]=checkLimits(theta_leg(:,1),q_lim_leg_R)
            [okL,breachedL]=checkLimits(theta_leg(:,2),q_lim_leg_L)
            [okRA,breachedRA]=checkLimits(theta_arm(:,1),q_lim_arm_R)
            [okLA,breachedLA]=checkLimits(theta_arm(:,2),q_lim_arm_L)
            if (okR && okL && okRA && okLA)
                
            else
                disp('joint limits breached')
            end
            %Avoiding limits for leg
            qd_rL=avoidJointLimits(t_leg_1(:,1),errorFi(:,1),J_leg_R,J_leg_inv(:,1:6),q_lim_leg_R,dt_leg(:,1));
            qd_lL=avoidJointLimits(t_leg_1(:,2),errorFi(:,2),J_leg_L,J_leg_inv(:,7:12),q_lim_leg_L,dt_leg(:,2));
            avoidLim=[qd_rL,qd_lL];
            theta_leg=theta_leg+avoidLim;
            %Avoiding limits for arm
            qd_rA=avoidJointLimits(t_arm_1(:,1),errorHi(:,1),J_arm_R,J_arm_inv(:,1:6),q_lim_arm_R,dt_arm(:,1));
            qd_lA=avoidJointLimits(t_arm_1(:,2),errorHi(:,2),J_arm_L,J_arm_inv(:,7:12),q_lim_arm_L,dt_arm(:,2));
            avoidLim=[qd_rA,qd_lA];
            theta_arm=theta_arm+avoidLim;
        end
        %%
        
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot(theta_leg,theta_arm,theta_head,swing(s),swing(s),s);
        % animate the motion
        if animation == 1
            animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
        %% Error in position
        p_Fi(:,1)=chain_leg_R(1:3,4,end);
        p_Fi(:,2)=chain_leg_L(1:3,4,end);
        p_Hi(:,1)=chain_arm_R(1:3,4,end);
        p_Hi(:,2)=chain_arm_L(1:3,4,end);
        p_B=T_global(1:3,4);
        
        errorFi(1:3,1)=dp_Fi(:,1)-p_Fi(:,1);
        errorFi(1:3,2)=dp_Fi(:,2)-p_Fi(:,2);
        errorHi(1:3,1)=dp_Hi(:,1)-p_Hi(:,1);
        errorHi(1:3,2)=dp_Hi(:,2)-p_Hi(:,2);
        errorB(1:3)=dp_B-p_B;
        %% Error in Orientation
       
        o_Fr = rotm2quat(chain_leg_R(1:3,1:3,end));
        o_Fl = rotm2quat(chain_leg_L(1:3,1:3,end));
        o_B = rotm2quat(T_global(1:3,1:3,end));
        
        % %  use with robotics toolbox installed
        if roboticTB==1
            errorFi(6:-1:4,1)=quat2eul(quatmultiply(quatinv(o_Fr),do_Fr));
            errorFi(6:-1:4,2)=quat2eul(quatmultiply(quatinv(o_Fl),do_Fl));
            errorB(6:-1:4)=quat2eul(quatmultiply(quatinv(o_B),do_B));
        end
        % use with aerospace toolbox installed
        if aerospaceTB==1
            errorFi(6:-1:4,1)=quat2angle(quatmultiply(quatinv(o_Fr),do_Fr));
            errorFi(6:-1:4,2)=quat2angle(quatmultiply(quatinv(o_Fl),do_Fl));
            errorB(6:-1:4)=quat2angle(quatmultiply(quatinv(o_B),do_B));
        end
        %only one joint contrls the yaw orientation for both legs which is not
        %engouh for controlling that orientation fully for both legs
        errorFi(6,1)=0;%leave the error in the yaw direction free
        errorFi(6,2)=0;
        errorB(6)=0;
        errorHi(4:6,1)=zeros(3,1); %orientation of the hand unavailable
        errorHi(4:6,2)=zeros(3,1);% all joint of hands still no activated
        %errorB(4:6)=zeros(3,1);
        %% Exit Criteria
        error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
        if error<refError
            best_thArm=theta_arm;
            best_thLeg=theta_leg;
            refError=error;
        end
        it=it+1;
        %     if it==30
        %
        % %               disp ( error )
        %     end
        %          disp (it)
        t_arm_1 = theta_arm;
        t_leg_1 = theta_leg;
    end
    %%
    if error<.0001
        %   disp ( strcat('converged with ',num2str(it),' iterations', ' point ',num2str(s ), ' of trajectory'))
        %   disp (error)
    else
        
        disp(strcat('no convergence after 500 iterations' , ' point ',num2str(s ), ' of trajectory'))
        errorFi
        errorHi
        errorB
    end
    %            theta_arm(:,:)-  t_arm_0
    %          theta_leg(:,:)-t_leg_0
    
    dtheta_leg=differentiate( best_thLeg,t_leg_0 );
    dtheta_arm=differentiate( best_thArm,t_arm_0 );
    
    if limit_vel == 1
        [dt_leg(:,1),okR] = velocity_check(dtheta_leg(:,1),max_v_leg);
        [dt_leg(:,2),okL ]= velocity_check(dtheta_leg(:,2),max_v_leg);
        [dt_arm(:,1),okRA] = velocity_check(dtheta_arm(:,1),max_v_arm);
        [dt_arm(:,2),okLA] = velocity_check(dtheta_arm(:,2),max_v_arm);
        %TODO: warnings for when an "optimal" value exceeds the vel limits
        if (okR && okL && okRA && okLA)
            
        else
            disp('joint velocity limits breached')
        end
    end

