function [theta_arm,theta_leg] = TestJacob (ddz_G, lambda_traj,p_k, alpha, n_k,t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG,theta_head,swing)
%WALKING_PATTERN_GENERATOR calculate the leg and arm joint velocities given
%references velocities of the hands, feet and waist and acceleration of the
%center of gravity of the robot
% Inputs:
%   xi_Fi_ref : velocity (linear and angular) of both feet
%   xi_Hi_ref : velocity (linear and angular) of both hands
%   xi_B_ref : velocity (linear and angular) of the waist
%   ddz_G : acceleration of the COG of the robot
%   lambda_traj : contact forces scalar
%   p_k : position of the contact points for the whole trajectory
%   alpha : variable that distributes normal forces and friction forces in the z
%       axis (when only feet in contact could be considered the average slope
%       of the ground) Note: might be reauired to have a different alpha value
%       for every step when walking on rough terrain, for flat ground can be
%       set to 0 for all trajectory
%   n_k: unit normal vector at every contact
%   t_arm_init : initial joint position of the arms
%   t_leg_init : initial joint position of the legs
%   P_COG : initial position of the center of mass

% Outputs:
%   dt_leg : joint velocities of the legs
%   dt_arm : joint velocities of the arms
%% Global variables
global epsilon;
global max_v_arm;
global max_v_leg;
global dt;
global animation

global T_global
[ x_c, y_c,z_c,T_Cx,T_Cy]=moment_vars(ddz_G, lambda_traj,p_k, alpha, n_k);

%% Walking Pattern Generator loop
% do the following steps while |v_B_ref-v_B|>epsilon
%% Initialisation of the velocity of the waist
% It is first initialised to a high value to start the algorithm corectly
v_B = ones(3,size(xi_B_traj,2))*1000; 
first_time = 1;
iteration=0;
xi_B=zeros(size(xi_B_traj));
    if first_time == 0
        % exchange the linear velocity given in reference with the new one found in
        % previous s
        xi_B_traj(1:3,:) = v_B;
       
    else
        first_time = 0;
    end
    %initial velocity 0
    dx_G0 = xi_B_traj(1,1);
    dy_G0 = xi_B_traj(2,1);
    dz_G0 = xi_B_traj(3,1);
    % initial angular momentum 0
    Lk0 = zeros(3,1);
    Lk1 = zeros(3,1);
    t_arm_0 = t_arm_init;
    t_leg_0 = t_leg_init;
    dL=zeros(3,size(xi_B_traj,2));
    L_ref=zeros(3,size(xi_B_traj,2));
    %MOVE to initial position again
    [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
        move_robot2(t_leg_init,t_arm_init,theta_head,swing(1),swing(1),1);
    %consider velocity limits? 0= no , 1=yes
    limit_vel=0;
    for s = 1:size(T_Cx,2)
        
        % extract the parameters at the time t
        xi_Fi = xi_Fi_traj(:,:,s);
        xi_Hi = xi_Hi_traj(:,:,s);
        xi_B_ref = xi_B_traj(:,s);
               
%         %Calculate desired positions
%         dp_Fi(:,1)=integrate(xi_Fi(1:3,1),chain_leg_R(1:3,4,end));
%          dp_Fi(:,2)=integrate(xi_Fi(1:3,2),chain_leg_L(1:3,4,end));
%          dp_Hi(:,1)=integrate(xi_Hi(1:3,1),chain_arm_R(1:3,4,end));
%           dp_Hi(:,2)=integrate(xi_Hi(1:3,2),chain_arm_L(1:3,4,end));
%           dp_B=integrate(xi_B_ref(1:3,1),T_global(1:3,4));
            
        %Calculate joint velocities from reference twists
        
%          [ dt_leg, dt_arm ] =foot_constraints( xi_B_ref, xi_Fi, xi_Hi,chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s );
    [ dt_leg, dt_arm,theta_leg, theta_arm ] = IK_Global( xi_B, xi_Fi, xi_Hi,chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s );

        if limit_vel == 1
            dt_leg(:,1) = velocity_check(dt_leg(:,1),max_v_leg);
            dt_leg(:,2) = velocity_check(dt_leg(:,2),max_v_leg);
            dt_arm(:,1) = velocity_check(dt_arm(:,1),max_v_arm);
            dt_arm(:,2) = velocity_check(dt_arm(:,2),max_v_arm);
        end
        % extract the vector of joint velocities
        dtheta = [dt_leg(:,1); dt_leg(:,2); dt_arm(:,1); dt_arm(:,2)];

%         global invJac
%         dtet=invJac*xi_Fi(1:3,2);
%         dt_leg=[dtet(6:-1:1),dtet(6:11)];
%         dt_arm=[dtet(12:15),dtet(17:20)];
        
        % integrate the joint velocities to get the angles values
        theta_arm(:,:,s) = integrate(dt_arm, t_arm_0);
        theta_leg(:,:,s) = integrate(dt_leg, t_leg_0);
        
        %ensure that the angular value is between [-pi/2;pi/2]
       theta_arm(:,:,s) = mod_interval(theta_arm(:,:,s),-2*pi,2*pi);
       theta_leg(:,:,s) = mod_interval(theta_leg(:,:,s),-2*pi,2*pi);
        
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
        if s+1<=size(T_Cx,2)
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
        else
            [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
            
        end
        % animate the motion
        if animation == 1
            animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
        
        
        % set the new value of the joint for the integrations
        t_arm_0 = theta_arm(:,:,s);
        t_leg_0 = theta_leg(:,:,s);
        Lk0 = Lk1;
        
        %Temporary code to check variations in the waist coordinates
        
        p_B(:,s)=T_global(1:3,4);
        p_Fi(:,s)=chain_leg_L(1:3,4,7);
        %sum(sum(abs(T_global(1:3,1:3))))
        if mod(s,40)==0
            azer=1;
        end
       
    end
       
    % calculate the x and y position of the COG in next time s by checking the CWS
    % stability criterion
%     figure, 
%     plot(L_ref(1,:),'r'); hold on;
%     plot(L_ref(2,:),'b'); hold on;
%     plot(L_ref(3,:),'g'); hold on;

    figure, 
%     plot(p_B(1,:),'r'); hold on;
%     plot(p_B(2,:),'b'); hold on;
    plot(p_B(3,:),'g'); hold on;
    %     plot(p_B(1,:),'r'); hold on;
%     plot(p_B(2,:),'b'); hold on;
    plot(p_Fi(3,:),'g'); hold on;
    
    figure,
    plot(squeeze(theta_leg(1,1,:)),'r'); hold on;
    plot(squeeze(theta_leg(2,1,:)),'b'); hold on;
    plot(squeeze(theta_leg(3,1,:)),'g'); hold on;
    plot(squeeze(theta_leg(4,1,:)),'c'); hold on;
    plot(squeeze(theta_leg(5,1,:)),'m'); hold on;
    plot(squeeze(theta_leg(6,1,:)),'k'); hold on;
    
    figure,
    plot(squeeze(theta_leg(1,2,:)),'r'); hold on;
    plot(squeeze(theta_leg(2,2,:)),'b'); hold on;
    plot(squeeze(theta_leg(3,2,:)),'g'); hold on;
    plot(squeeze(theta_leg(4,2,:)),'c'); hold on;
    plot(squeeze(theta_leg(5,2,:)),'m'); hold on;
    plot(squeeze(theta_leg(6,2,:)),'k'); hold on;
    
      
 