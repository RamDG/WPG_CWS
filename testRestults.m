function []= testRestults(ddz_G, lambda_traj,p_k, alpha, n_k,theta_arm,theta_leg,xi_Fi_traj,xi_Hi_traj,xi_B_traj,P_COG_init,theta_head,swing)
%global animation
animation=1;
global T_global
global P_COG
[ x_c, y_c,z_c,T_Cx,T_Cy]=moment_vars(ddz_G, lambda_traj,p_k, alpha, n_k);
dtheta_leg=zeros(size(theta_leg));
dtheta_arm=zeros(size(theta_arm));
dtheta_leg(:,:,2:end)=differentiate( theta_leg(:,:,2:end),theta_leg(:,:,1:end-1) );
dtheta_arm(:,:,2:end)=differentiate( theta_arm(:,:,2:end),theta_arm(:,:,1:end-1) );
 %% calculate  WS at initial position assuming double support 
global total_mass g
   wsx= total_mass*g*P_COG_init(2);
   wsy= -total_mass*g*P_COG_init(1);
   
   T_Cx(1,1)=wsx;
   T_Cy(1,1)=wsy;
   
    % initial angular momentum 0
    Lk0 = zeros(3,1);
    Lk1 = zeros(3,1);
    % initial values for angular momentum
    dL=zeros(3,size(xi_B_traj,2));
    L_ref=zeros(3,size(xi_B_traj,2));
    %MOVE to initial position again
    
    move_robot(theta_leg(:,:,1),theta_arm(:,:,1),theta_head,swing(1),swing(1),1);
    % move_robot2(theta_leg(:,:,1),theta_arm(:,:,1),theta_head,swing(1),swing(1),1);
    % initial values for acc and v of COG
    pc=P_COG_init;
    dpc=0;
    frames=struct('cdata', cell(1,size(T_Cx,2)), 'colormap', cell(1,size(T_Cx,2)));
 for s = 1:size(xi_B_traj,2)
        
        % extract the parameters at the time t
        xi_Fi = xi_Fi_traj(:,:,s);
        xi_Hi = xi_Hi_traj(:,:,s);
        xi_B_ref = xi_B_traj(:,s);
       dt_leg=dtheta_leg(:,:,s);
       dt_arm=dtheta_arm(:,:,s);
        % extract the vector of joint velocities
        dtheta = [dt_leg(:,1); dt_leg(:,2); dt_arm(:,1); dt_arm(:,2)];
        
        
        %Calculate angular momenntum
        Lk1 = angular_momentum(xi_B_ref, dtheta);
        [~,L_comp(:,s)] = angular_momentum2(xi_B_ref, xi_Fi, xi_Hi);
        L_ref(:,s)=Lk1;
        % differentiate it to get dL_ref
        dL_ref = differentiate(Lk1, Lk0);
        %store information on the derivative of angular momentum
        dL(:,s)=dL_ref;
        
     
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
         if s+1<=size(xi_B_traj,2)
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
        else
            [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
            
         end
%         if s+1<=size(xi_B_traj,2)
%         [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
%             move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s+1),s);
%         else
%             [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
%             move_robot2(theta_leg(:,:,s),theta_arm(:,:,s),theta_head,swing(s),swing(s),s);
%             
%         end
        % animate the motion
        if animation == 1
            frames(s)=animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
        end
        if s==100
           stop=1; 
        end
        
        % set the new value for the integrations
           Lk0 = Lk1;
        
        %Temporary code to check variations in the waist coordinates
        
        p_B(:,s)=T_global(1:3,4);
        p_C(:,s)=P_COG;
        p_C(3,s)=P_COG_init(3);
        dp_C(:,s)=differentiate(p_C(:,s),pc);
        ddp_C(:,s)=differentiate(dp_C(:,s),dpc);
        
        WrenchSum(:,s)=wrench_sum(p_C(:,s),ddp_C(:,s),dL(:,s));
        
        pc=p_C(:,s);
        dpc=dp_C(:,s);
        
        
        
         uthetaR = vrrotmat2vec(chain_leg_R(1:3,1:3,end));
         o_Fi(:,1,s) = [uthetaR(1); uthetaR(2); uthetaR(3)]*uthetaR(4);
      uthetaL = vrrotmat2vec(chain_leg_L(1:3,1:3,end));
        o_Fi(:,2,s) = [uthetaL(1); uthetaL(2); uthetaL(3)]*uthetaL(4);
       
        %sum(sum(abs(T_global(1:3,1:3))))
 end
    
      %% plots for debugging
      %Orientation
          figure,
%       plot(squeeze(o_Fi(1,1,:)),'r'); hold on;
%       plot(squeeze(o_Fi(2,1,:)),'b'); hold on;
%       plot(squeeze(o_Fi(3,1,:)),'g'); hold on;  
% 
%       plot(squeeze(o_Fi(1,2,:)),'c'); hold on;
%       plot(squeeze(o_Fi(2,2,:)),'m'); hold on;
%       plot(squeeze(o_Fi(3,2,:)),'k'); hold on;  
      
%       Angular momentum and derivative
%
%       figure,
%       plot(L_ref(1,:),'r'); hold on;
%       plot(L_ref(2,:),'b'); hold on;
%       plot(L_ref(3,:),'g'); hold on;
%       
%       figure,
%       plot(L_comp(1,:),'r'); hold on;
%       plot(L_comp(2,:),'b'); hold on;
%       plot(L_comp(3,:),'g'); hold on;
%       
%       figure,plot(L_ref(1,:)-L_comp(1,:))
%       figure,plot(L_ref(2,:)-L_comp(2,:))
%       figure,plot(L_ref(3,:)-L_comp(3,:))
%       
      figure,
      plot(dL(1,:),'r'); hold on;
      plot(dL(2,:),'b'); hold on;
      plot(dL(3,:),'g'); hold on;
%     
    % CWS vs CWC
      figure,
      plot(T_Cx,'r');hold on;
      plot(WrenchSum(1,:));hold on;
     legend('T_Cx_ref','T_Cx',2);
      xlabel('Time [sec]');
      ylabel('T_Cx, N*meters')
      
      figure,
      plot(T_Cy,'r');hold on;
      plot(WrenchSum(2,:));hold on;
      legend('T_Cy_ref','T_Cy',2);
      xlabel('Time [sec]');
      ylabel('T_Cy, N*meters')
      
  
%        figure,
%       plot(T_Cx-WrenchSum(1,:),'r');hold on;
%      legend('Diff in T_Cx',2);
%       xlabel('Time [sec]');
%       ylabel('T_Cx, N*meters')
%       
%       figure,
%       plot(T_Cy-WrenchSum(2,:),'r');hold on;
%       legend('Diff in T_Cy',2);
%       xlabel('Time [sec]');
%       ylabel('T_Cy, N*meters')

      %Waist
%       figure,
%       plot(p_B(1,:),'r'); hold on;
%       plot(p_B(2,:),'b'); hold on;
%       plot(p_B(3,:),'g'); hold on;
%      
%         dp_B=zeros(size(p_B));
%         dp_B(:,2:end)=differentiate(p_B(:,2:end),p_B(:,1:end-1));

%           figure,
%       plot(dp_B(1,:),'r'); hold on;
%       plot(dp_B(2,:),'b'); hold on;
%       plot(dp_B(3,:),'g'); hold on;
% 
%     figure, 
%     plot(xi_B_traj(1,:),'r');
%     hold on;
%     plot(xi_B_traj(2,:),'b');
%     hold on;
%     plot(xi_B_traj(3,:),'g');

%           figure,
%       plot(dp_B(1,:),'r'); hold on;
%     plot(xi_B_traj(1,:),'b');
% 
%     figure, 
%       plot(dp_B(2,:),'r'); hold on;
%     plot(xi_B_traj(2,:),'b');
% 
% 
% 
%     figure, 
%       plot(dp_B(3,:),'r'); hold on;
%     plot(xi_B_traj(3,:),'g');
% 
% 
% %       CoG
%       figure,
%       plot(p_C(1,:),'r'); hold on;
%       plot(p_C(2,:),'b'); hold on;
%       plot(p_C(3,:),'g'); hold on;
%       
%        figure,
%       plot(dp_C(1,:),'r'); hold on;
%       plot(dp_C(2,:),'b'); hold on;
%       plot(dp_C(3,:),'g'); hold on;

      % Joints
%       figure,
%       plot(squeeze(theta_leg(1,1,:)),'r'); hold on;
%       plot(squeeze(theta_leg(2,1,:)),'b'); hold on;
%       plot(squeeze(theta_leg(3,1,:)),'g'); hold on;
%       plot(squeeze(theta_leg(4,1,:)),'c'); hold on;
%       plot(squeeze(theta_leg(5,1,:)),'m'); hold on;
%       plot(squeeze(theta_leg(6,1,:)),'k'); hold on;
%       
%       figure,
%       plot(squeeze(theta_leg(1,2,:)),'r'); hold on;
%       plot(squeeze(theta_leg(2,2,:)),'b'); hold on;
%       plot(squeeze(theta_leg(3,2,:)),'g'); hold on;
%       plot(squeeze(theta_leg(4,2,:)),'c'); hold on;
%       plot(squeeze(theta_leg(5,2,:)),'m'); hold on;
%       plot(squeeze(theta_leg(6,2,:)),'k'); hold on;
%       

%% Now test preview control
 [ TC, x, y ] = solveMomentEq( P_COG_init(3), ddz_G,  dL, x_c, y_c,z_c,T_Cx,T_Cy,P_COG_init );
 
%   figure,
%       plot(T_Cx,'r');hold on;
%       plot(TC(1,:)+dL(1,:),'b');hold on;
%     %  plot(WrenchSum(1,:));hold on;
%      legend('T_Cx_ref','T_Cx',2);
%       xlabel('Time [sec]');
%       ylabel('T_Cx, N*meters')
%       
%   figure,
%       plot(T_Cy,'r');hold on;
%       plot(TC(2,:)+dL(2,:),'b');hold on;
%       %plot(WrenchSum(2,:));hold on;
%       legend('T_Cy_ref','T_Cy',2);
%       xlabel('Time [sec]');
%       ylabel('T_Cy, N*meters')
%       
%       
%       figure,
%       plot(p_C(1,:),'r'); hold on;
%       plot(x(:,1),'b');
%       figure,
%       plot(p_C(2,:),'r'); hold on;
%       plot(y(:,1),'b');
%       
%        figure,
%       plot(dp_C(1,:),'r'); hold on;
%       plot(x(:,2),'b');
%       figure,
%       plot(dp_C(2,:),'r'); hold on;
%       plot(y(:,2),'b');
%       
%        figure,
%       plot(p_C(1,:)-x(:,1)','r'); hold on;
%      
%       figure,
%       plot(p_C(2,:)-y(:,1)','b'); hold on;
%      
%       
%        ddp_Cc=[0;0;0];
%     ws=zeros(2,201);
%       for s=2:size(dL,2)
%         p_Cc=[x(s,1);y(s,1);P_COG_init(3)];
%         ddp_Cc(1,s)=differentiate(x(s,2),x(s-1,2));
%         ddp_Cc(2,s)=differentiate(y(s,2),y(s-1,2));
%         ddp_Cc(3,s)=0;
%         ws(:,s)=wrench_sum(p_Cc,ddp_Cc(:,s),dL(:,s));
% %         plot(ddp_Cc(1),'*');hold on;
% %         plot(ddp_Cc(2),'*');hold on;
%     end
%     figure,
%     plot(ws(1,:))
%     figure,
%     plot(ws(2,:))
 
   %% Write movie file
   %writerObj = VideoWriter('cws','MPEG-4');
    writerObj = VideoWriter('cws');
open(writerObj);
for s = 1:size(T_Cx,2)
 writeVideo(writerObj,frames(s));
end
close(writerObj);
end
