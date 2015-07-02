global T_transl

if swing_foot==2
    q=[theta_leg(6,1),theta_leg(5,1),theta_leg(4,1),theta_leg(3,1),theta_leg(2,1),theta_leg(1,1),...
        theta_leg(2:end,2)',theta_arm(:,1)',0,theta_arm(:,2)',0,theta_head'];
    if s==1
        T_transl=eye(4);
        if isa(q,'sym')
            T_transl=sym(T_transl);
        end
    end
    
    PARAMETERS_RFoot
    if s~=1
    chain_leg_R(:,:,1)=T(:,:,6);
    chain_leg_R(:,:,2)=T(:,:,5);
    chain_leg_R(:,:,3)=T(:,:,4);
    chain_leg_R(:,:,4)=T(:,:,3);
    chain_leg_R(:,:,5)=T(:,:,2);
    chain_leg_R(:,:,6)=T(:,:,1);
    chain_leg_R(:,:,7)=T_transl;
    chain_leg_L=T(:,:,8:14);
    chain_arm_R=T(:,:,15:18);
    chain_arm_R=cat(3,chain_arm_R,T(:,:,20));
    chain_arm_L=T(:,:,21:24);
    chain_arm_L=cat(3,chain_arm_L,T(:,:,26));
    chain_head=T(:,:,27:28);
    end
end

if swing_foot==1
    q=[theta_leg(6,2),theta_leg(5,2),theta_leg(4,2),theta_leg(3,2),theta_leg(2,2),theta_leg(1,2),...
        theta_leg(2:end,1)',theta_arm(:,1)',0,theta_arm(:,2)',0,theta_head'];
    if s==1
        T_transl=eye(4);
        if isa(q,'sym')
            T_transl=sym(T_transl);
        end
    end
    
    PARAMETERS_LFoot
    if s~=1
    chain_leg_L(:,:,1)=T(:,:,6);
    chain_leg_L(:,:,2)=T(:,:,5);
    chain_leg_L(:,:,3)=T(:,:,4);
    chain_leg_L(:,:,4)=T(:,:,3);
    chain_leg_L(:,:,5)=T(:,:,2);
    chain_leg_L(:,:,6)=T(:,:,1);
    chain_leg_L(:,:,7)=T_transl;
    chain_leg_R=T(:,:,8:14);
    chain_arm_R=T(:,:,15:18);
    chain_arm_R=cat(3,chain_arm_R,T(:,:,20));
    chain_arm_L=T(:,:,21:24);
    chain_arm_L=cat(3,chain_arm_L,T(:,:,26));
    chain_head=T(:,:,27:28);
    end
    
end
 T_global=T(:,:,7);

if ( s==1)
    if swing_foot==2
        
        d_R=(T_global(1:3,4));
        %In the beginning Set global reference below the waist at ground
        %level
        T_transl=makehgtformS('translate',[-d_R(1:2);0]);
        
    else
        d_L=(T_global(1:3,4));
        %In the beginning Set global reference below the waist at ground
        %level
        T_transl=makehgtformS('translate',[-d_L(1:2);0]);
        
    end
    for j = 1:length(a);
        T(:,:,j) = T_transl* T(:,:,j);
    end
     if swing_foot==2
         chain_leg_R(:,:,1)=T(:,:,6);
    chain_leg_R(:,:,2)=T(:,:,5);
    chain_leg_R(:,:,3)=T(:,:,4);
    chain_leg_R(:,:,4)=T(:,:,3);
    chain_leg_R(:,:,5)=T(:,:,2);
    chain_leg_R(:,:,6)=T(:,:,1);
    chain_leg_R(:,:,7)=T_transl;
    chain_leg_L=T(:,:,8:14);
    chain_arm_R=T(:,:,15:18);
    chain_arm_R=cat(3,chain_arm_R,T(:,:,20));
    chain_arm_L=T(:,:,21:24);
    chain_arm_L=cat(3,chain_arm_L,T(:,:,26));
    chain_head=T(:,:,27:28);
      
    else
       chain_leg_L(:,:,1)=T(:,:,6);
    chain_leg_L(:,:,2)=T(:,:,5);
    chain_leg_L(:,:,3)=T(:,:,4);
    chain_leg_L(:,:,4)=T(:,:,3);
    chain_leg_L(:,:,5)=T(:,:,2);
    chain_leg_L(:,:,6)=T(:,:,1);
    chain_leg_L(:,:,7)=T_transl;
    chain_leg_R=T(:,:,8:14);
    chain_arm_R=T(:,:,15:18);
    chain_arm_R=cat(3,chain_arm_R,T(:,:,20));
    chain_arm_L=T(:,:,21:24);
    chain_arm_L=cat(3,chain_arm_L,T(:,:,26));
    chain_head=T(:,:,27:28);
    end
    T_global=T(:,:,7);

end

change=swing_foot-nextSwing_foot;
if (change~=0)
    if(change <0)
        T_transl=chain_leg_R(:,:,end);
    else
        T_transl=chain_leg_L(:,:,end);
    end
end
