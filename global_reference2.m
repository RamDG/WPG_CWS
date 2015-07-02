function [chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head]=global_reference2(chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head,swing_foot,s)
%Create a global reference
%Original Reference is on the waist
%We want the feet to be in the ground so the orientation should come from
%the feet
% z_B=-z_F
%x and y free to choose?
%one we have coordinates and orientation create transformation matrix and
%then premultiply all the chains to shift the reference to the global frame
global T_global
global T_transl
change=mod(s,20);

if (change==1 || s==1)
   if swing_foot==2
        R=chain_leg_R(1:3,1:3,end); %orientation of the end effector of the right foot with respect to the waist
         d_R=-R'*chain_leg_R(1:3,4,end);
         p=d_R;
         if s==1
         T_transl=makehgtformS('translate',[-d_R(1:2);0]);
         else
              T_transl=T_global*chain_leg_R(:,:,end);
         end
    else
         
        R=chain_leg_L(1:3,1:3,end); %orientation of the end effector of the right foot with respect to the waist
        d_L=-R'*chain_leg_L(1:3,4,end);
        p=d_L;
        if s==1
         T_transl=makehgtformS('translate',[-d_L(1:2);0]);
         else
              T_transl=T_global*chain_leg_L(:,:,end);
         end
    end
else
     if swing_foot==2
        R=chain_leg_R(1:3,1:3,end); %orientation of the end effector of the right foot with respect to the waist
        p=-R'*chain_leg_R(1:3,4,end);% 
       
    else
        R=chain_leg_L(1:3,1:3,end); %orientation of the end effector of the right foot with respect to the waist
        p=-R'*chain_leg_L(1:3,4,end);% 
        
    end
end
if  isa(R,'sym')
  R= vpa(R,8) ;
  p= vpa(p,8) ;
  T_transl= vpa(T_transl,8) ;
  end
T(1:4,1:3)=[R';[0,0,0]];%reverse orientation assuming the foot is aligned with the floor
T(1:4,4)=[p;1];
T=T_transl*T;


for i = 1:size(chain_leg_R,3)
    % get the transformation matrix of the i-th link
    chain_leg_R(:,:,i)=T*chain_leg_R(:,:,i);
    chain_leg_L(:,:,i)=T*chain_leg_L(:,:,i);
end

for i = 1:size(chain_arm_R,3)
    % get the transformation matrix of the i-th link
    chain_arm_R(:,:,i)=T*chain_arm_R(:,:,i);
    chain_arm_L(:,:,i)=T*chain_arm_L(:,:,i);
end

for i = 1:size(chain_head,3)
    % get the transformation matrix of the i-th link
    chain_head(:,:,i)=T*chain_head(:,:,i);
    
end
T_global=T;
%T_global(1:3,4)
%NOTE: this would require a change in the inertia calculation to include
%the rotation matrix with respect to global base at the calculation of
%inertia total matrix
