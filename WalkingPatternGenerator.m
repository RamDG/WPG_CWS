function [theta_arm,theta_leg] = WalkingPatternGenerator (ddz_G, lambda_traj,p_k, alpha, n_k,t_arm_init,t_leg_init,xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG_init,theta_head,swing)
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
global P_COG
[ x_c, y_c,z_c,T_Cx,T_Cy]=moment_vars(ddz_G, lambda_traj,p_k, alpha, n_k);
%%Choose which jacobian to use
%separateChains=0;
global separateChains
global timings;
%% calculate  WS at initial position assuming double support
global total_mass g
wsx= total_mass*g*P_COG_init(2);
wsy= -total_mass*g*P_COG_init(1);

T_Cx(1,1)=wsx;
T_Cy(1,1)=wsy;

%% Walking Pattern Generator loop
% do the following steps while |v_B_ref-v_B|>epsilon
%% Initialisation of the velocity of the waist
% It is first initialised to a high value to start the algorithm corectly
v_B = ones(3,size(xi_B_traj,2))*1000;
first_time = 1;
iteration=0;
xi_B=zeros(size(xi_B_traj));

startTime=tic;
while (sum(sum(abs(xi_B_traj(1:3,:)-v_B))>epsilon))>0
    if first_time == 0
        % exchange the linear velocity given in reference with the new one found in
        % previous s
        xi_B_traj(1:3,:) = v_B;
        %xi_B_traj(1:2,:) =v_B(1:2,:);
        xi_Hi_traj(1:2,1,:) =v_B(1:2,:);
        xi_Hi_traj(1:2,2,:) =v_B(1:2,:);
        
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
    % initial values of joints
    t_arm_0 = t_arm_init;
    t_leg_0 = t_leg_init;
    % initial values for angular momentum
    dL=zeros(3,size(xi_B_traj,2));
    L_ref=zeros(3,size(xi_B_traj,2));
    %MOVE to initial position again
    if separateChains==1
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot(t_leg_init,t_arm_init,theta_head,swing(1),swing(1),1);
    else
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot2(t_leg_init,t_arm_init,theta_head,swing(1),swing(1),1);
    end
    
    % initial values for acc and v of COG
    pc=P_COG_init;
    dpc=0;
    ddpc=0;
    %consider velocity limits? 0= no , 1=yes
    limit_vel=0;
    for s = 1:size(T_Cx,2)
        
        % extract the parameters at the time t
        xi_Fi = xi_Fi_traj(:,:,s);
        xi_Hi = xi_Hi_traj(:,:,s);
        xi_B_ref = xi_B_traj(:,s);
        
        %Calculate joint velocities from reference twists (inverse
        %Kinematics)
        if separateChains==1
            [ dt_leg, dt_arm,t_leg, t_arm,it(s) ] =foot_constraints( xi_B_ref, xi_Fi, xi_Hi,...
                chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s );
        else
            [ dt_leg, dt_arm,t_leg, t_arm,it(s) ] =IK_Global( xi_B_ref, xi_Fi, xi_Hi,...
                chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s );
        end
        
        if limit_vel == 1
            [dt_leg(:,1),okR] = velocity_check(dtheta_leg(:,1),max_v_leg);
            [dt_leg(:,2),okL ]= velocity_check(dtheta_leg(:,2),max_v_leg);
            [dt_arm(:,1),okRA] = velocity_check(dtheta_arm(:,1),max_v_arm);
            [dt_arm(:,2),okLA] = velocity_check(dtheta_arm(:,2),max_v_arm);
            %warnings for when an "optimal" value exceeds the vel limits
            if (okR && okL && okRA && okLA)
                
            else
                disp(strcat('joint velocity limits breached, ',num2str(s ), ' of trajectory'))
                
            end
        end
        % extract the vector of joint velocities
        dtheta = [dt_leg(:,1); dt_leg(:,2); dt_arm(:,1); dt_arm(:,2)];
        
        
        %Calculate angular momenntum
        Lk1 = angular_momentum(xi_B_ref, dtheta);
        L_ref(:,s)=Lk1;
        % differentiate it to get dL_ref
        dL_ref = differentiate(Lk1, Lk0);
        %store information on the derivative of angular momentum
        dL(:,s)=dL_ref;
        
        theta_arm(:,:,s) = t_arm;
        theta_leg(:,:,s) = t_leg;
        
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
        if separateChains==1
            if s+1<=size(T_Cx,2)
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
            else
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
                
            end
        else
            if s+1<=size(T_Cx,2)
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
            else
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
                
            end
            
        end
        % animate the motion
        if animation == 1
            animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
        
        
        % set the new value of the joint for the integrations
        t_arm_0 = theta_arm(:,:,s);
        t_leg_0 = theta_leg(:,:,s);
        Lk0 = Lk1;
        
        %Temporary code to check variables
        %         [~,L_comp(:,s)] = angular_momentum2(xi_B_ref, xi_Fi, xi_Hi);
        %         p_B(:,s)=T_global(1:3,4);
        %         p_C(:,s)=P_COG;
        %          p_C(3,s)=P_COG_init(3);
        %         dp_C(:,s)=differentiate(p_C(:,s),pc);
        %         ddp_C(:,s)=differentiate(dp_C(:,s),dpc);
        %
        %         WrenchSum(:,s)=wrench_sum(p_C(:,s),ddp_C(:,s),dL(:,s));
        %
        %         pc=p_C(:,s);
        %         dpc=dp_C(:,s);
        %         %sum(sum(abs(T_global(1:3,1:3))))
                if s==100
                   disp ('its checking time')
                end
        %
        
    end
    %% Info for debugging
    mean(it)
    disp('average number of iterations')
    mean(timings)
    disp('average inverting jacobian time')
    if separateChains==1
        
            disp('Using a jacobian per limb')
    else
        
           disp('Using a jacobian for the whole body')
    end
    toc(startTime)
    %Can be taken from TestResults
    %%
    if separateChains==1
        
            move_robot(t_leg_init,t_arm_init,theta_head,swing(1),swing(1),1);
    else
        
            move_robot2(t_leg_init,t_arm_init,theta_head,swing(1),swing(1),1);
    end
    % calculate the x and y position of the COG in next time s by checking the CWS
    % stability criterion
    [ TC, x, y ] = solveMomentEq( P_COG_init(3), ddz_G,  dL, x_c, y_c,z_c,T_Cx,T_Cy,P_COG_init );
    
    for s = 1:size(T_Cx,2)
        % extract the parameters at the time t
        xi_Fi = xi_Fi_traj(:,:,s);
        xi_Hi = xi_Hi_traj(:,:,s);
        
        %integrate the acceleration on z to get velocity
        dz_G1 = integrate(ddz_G(s), dz_G0);
        %     if s>1
        %     dx_G1 = differentiate(x(s,1),x(s-1,1));
        %     dy_G1 = differentiate(y(s,1),y(s-1,1));
        %     else
        %    dx_G1 =0;
        %     dy_G1 = 0;
        %     end
        
        % calculate P_ref, momentum of the robot
        P_ref = momentum([x(s,2); y(s,2); dz_G1]);
        %     P_ref = momentum([dx_G1; dy_G1; dz_G1]);
        %TODO: possibility to use P_z from eq 38 , Hirukawa et al. 2006
        % find the new waist velocity
        if separateChains==1
            if s+1<=size(T_Cx,2)
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
            else
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
                
            end
        else
            if s+1<=size(T_Cx,2)
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
            else
                [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
                    move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
                
            end
            
        end
        xi_B(:,s) = resolved_momentum_control(P_ref,L_ref(:,s),xi_Fi,xi_Hi);
        
        dz_G0 = dz_G1;
        
    end
    % extract the linear velocity
    %v_B = [xi_B(1:2,:);zeros(size(xi_B(1,:)))];
    v_B = xi_B(1:3,:);
    
    %     figure,
    %     plot(xi_B(1,:),'r');  hold on;
    %     plot(xi_B(2,:),'b');  hold on;
    %     plot(xi_B(3,:),'g');  hold off;
    %     figure,
    %     plot(xi_B(4,:),'r');  hold on;
    %     plot(xi_B(5,:),'b');  hold on;
    %     plot(xi_B(6,:),'g');  hold off;
    %     figure,plot(xi_B(1,:),'g')
    iteration=iteration+1
    sum(sum(abs(xi_B_traj(1:3,:)-v_B)))
    sum(sum(abs(xi_B(4:6,:))))
    savefile='results/VB.mat';
    save(savefile,'v_B');
    toc(startTime)
    %     ddp_Cc=[0;0;0];
    %     ws=zeros(2,201);
    %       for s=2:size(dL,2)
    %         p_C=[x(s,1);y(s,1);P_COG_init(3)];
    %         ddp_Cc(1,s)=differentiate(x(s,2),x(s-1,2));
    %         ddp_Cc(2,s)=differentiate(y(s,2),y(s-1,2));
    %         ddp_Cc(3,s)=0;
    %         ws(:,s)=wrench_sum(p_C,ddp_Cc(:,s),dL(:,s));
    % %         plot(ddp_Cc(1),'*');hold on;
    % %         plot(ddp_Cc(2),'*');hold on;
    %     end
    %     figure,
    %     plot(ws(1,:))
    %     figure,
    %     plot(ws(2,:))
    %     figure,
    %     plot(ddp_Cc(2,:));hold on;
    
end


mean(it)
disp('average number of iterations')
mean(timings)
disp('average inverting jacobian time')

end
