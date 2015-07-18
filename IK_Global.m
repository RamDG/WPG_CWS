function [ dtheta_leg, dtheta_arm,theta_leg, theta_arm,it ] = IK_Global( xi_B, xi_Fi, xi_Hi,chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, theta_head,t_arm_0,t_leg_0,swing,s )
%Calculate joint velocities from reference twists

%REFERENCE_ANGULAR_MOMENTUM calculate the joint velocities for the arm and
%the legs
%   eps_B : velocities of the waist
%   eps_Fi : velocities of the feet
%   eps_Hi : velocities of the hands

%   dtheta_leg : joint velocities of the legs
%   dtheta_arm : joint velocities of the arms

%% Declaration of the global variables
global Jac invJac;
global T_global 
%%
limits=0;
limit_vel=0;
animation=1;
% Dependency on matlab toolboxes
%% For limit checking
if limits==1
global q_lim_leg_R q_lim_leg_L q_lim_arm_R q_lim_arm_L %q_lim_head
global J_arm_R J_arm_L J_leg_R J_leg_L
end
global control cc timeDT
 controlVector=[zeros(1,3),zeros(1,3),ones(1,3),[1,1,1],zeros(1,3),zeros(1,3),zeros(1,3),zeros(1,3),];
    %cMatrix=diag(controlVector);
    cMatrix=diag(control);

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



    o_Fr = rotm2quat(chain_leg_R(1:3,1:3,end));
    o_Fl = rotm2quat(chain_leg_L(1:3,1:3,end));
    % o_Hi(:,1) = rotm2quat(chain_arm_R(1:3,1:3,end));
    % o_Hi(:,2) = rotm2quat(chain_arm_L(1:3,1:3,end));
    o_B = rotm2quat(T_global(1:3,1:3,end));
    
    % %  use with robotics toolbox installed
     if roboticTB==1
             errorFi(6:-1:4,1)=quat2eul(quatmultiply(quatinv(o_Fr),do_Fr));
             errorFi(6:-1:4,2)=quat2eul(quatmultiply(quatinv(o_Fl),do_Fl));
             if swing(s)==2
                 errorB(6:-1:4)=-quat2eul(quatmultiply(quatinv(o_B),do_B));
             else
                 
                 errorB(6:-1:4)=quat2eul(quatmultiply(quatinv(o_B),do_B));
             end
         end
         % use with aerospace toolbox installed
         if aerospaceTB==1
             errorFi(6:-1:4,1)=quat2angle(quatmultiply(quatinv(o_Fr),do_Fr));
             errorFi(6:-1:4,2)=quat2angle(quatmultiply(quatinv(o_Fl),do_Fl));
             if swing(s)==2
                 errorB(6:-1:4)=-quat2angle(quatmultiply(quatinv(o_B),do_B));
             else
                 
                 errorB(6:-1:4)=quat2angle(quatmultiply(quatinv(o_B),do_B));
             end
         end
    errorFi(6,1)=0;%leave the error in the yaw direction free
    errorFi(6,2)=0;
    %errorB(6)=0;
    %Dont control hand orientation
    errorHi(4:6,1)=zeros(3,1);
    errorHi(4:6,2)=zeros(3,1);
    %errorB(4:6)=zeros(3,1);
    errorB=errorB';
    
    %error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
    errorA=[errorFi(:,swing(1));errorB;errorHi(:,1);errorHi(:,2)];
        error=sum(sum(abs(cMatrix*errorA)));
    refError=99;
    best_thArm=t_arm_0;
    best_thLeg=t_leg_0;
    t_arm_1 =t_arm_0;
    t_leg_1 =t_leg_0;
    theta_arm(:,:) = t_arm_1;
    theta_leg(:,:) =t_leg_1;
    it=0;
    
   
    while (sum(abs(error))>.0001 && it<700)
        tsart=tic;
       %         global invJac
       if swing(s)==2
           errorAll=[errorFi(:,2);errorB;errorHi(:,1);errorHi(:,2)];
                dt_leg=[dtet(6:-1:1),dtet(6:11)];
                dt_arm=[dtet(12:15),dtet(17:20)];
       else
            errorAll=[errorFi(:,1);errorB;errorHi(:,1);errorHi(:,2)];
                dt_leg=[dtet(6:11),dtet(6:-1:1)];
                dt_arm=[dtet(12:15),dtet(17:20)];
       end
       
       lambda=1;
        % integrate the joint velocities to get the angles values
        theta_arm = lambda*dt_arm+ t_arm_1;
        theta_leg = lambda*dt_leg+ t_leg_1;
        
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
            if swing(s)==2
                %Avoiding limits for right leg support
                theta_1=[reshape(t_leg_1,1,[]),reshape(t_arm_1,1,[])];%TODO: restructure current values of joints into one vector
                dtheta=[reshape(dt_leg,1,[]),reshape(dt_arm,1,[])];%Reestructure delta theta to one vector
                qd_rL=avoidJointLimits(theta_1,errorAll,Jac,invJac,[q_lim_leg_R; q_lim_leg_L; q_lim_arm_R; q_lim_arm_L],dtheta);
                qt_leg=[qd_rL(6:-1:1),qd_rL(6:11)];
                qt_arm=[qd_rL(12:15),qd_rL(17:20)];
            else
                %Avoiding limits for left leg support
                theta_1=[reshape(t_leg_1,1,[]),reshape(t_arm_1,1,[])];%TODO: restructure current values of joints into one vector
                dtheta=[reshape(dt_leg,1,[]),reshape(dt_arm,1,[])];%Reestructure delta theta to one vector
                qd_lL=avoidJointLimits(theta_1(:,2),errorAll(:,2),Jac,invJac,[q_lim_leg_R; q_lim_leg_L; q_lim_arm_R; q_lim_arm_L],dtheta);
                qt_leg=[qd_lL(6:11),qd_lL(6:-1:1)];
                qt_arm=[qd_lL(12:15),qd_lL(17:20)];
            end
        end
        %%
        timeDT(cc)=toc(tsart);
        cc=cc+1;
        % move the robot with the new desired joint values, update
        % jacobians and intertia matrices
        [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
            move_robot2(theta_leg,theta_arm,theta_head,swing(s),swing(s),s);
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
% %          errorFi(6,1)=0;%leave the error in the yaw direction free
% %         errorFi(6,2)=0;
%         %errorB(6)=0;
%         errorHi(4:6,1)=zeros(3,1); %orientation of the hand unavailable
%         errorHi(4:6,2)=zeros(3,1);% all joint of hands still no activated
%         %errorB(4:6)=zeros(3,1);
%         %% Exit Criteria
%         sum(abs(errorB));
        %error=sum(sum(abs(errorFi)))+sum(sum(abs(errorHi)))+sum(abs(errorB));
        errorA=[errorFi(:,swing(s));errorB;errorHi(:,1);errorHi(:,2)];
        ee=cMatrix*errorA;
        error=sum(sum(abs(cMatrix*errorA)));
        e(it+1)=error;
        eB(:,it+1)=ee(7:12);
        eF(:,it+1)=ee(1:6);
        eH1(:,it+1)=ee(13:18);
        eH2(:,it+1)=ee(19:24);
        
        if error>1
            disp('failing')
        end
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
%           disp ( strcat('converged with ',num2str(it),' iterations', ' point ',num2str(s ), ' of trajectory'))
%           disp (error)
    else
        
        disp(strcat('no convergence after 500 iterations' , ' point ',num2str(s ), ' of trajectory'))
        errorFi
        errorHi
        errorB
    end
    %            theta_arm(:,:)-  t_arm_0
    %          theta_leg(:,:)-t_leg_0
    if(it>0)
    global Test;
    savefile=sprintf('results/JacobiansError/errors%d_test%dTS.mat',s,Test);
    save(savefile,'e','eB','eF','eH1','eH2');
    end
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

