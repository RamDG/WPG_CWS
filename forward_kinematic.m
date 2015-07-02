function [ T_end, T_chain ] = forward_kinematic( alpha, d, theta, r, t_base, t_end, rot, rot_fix )
%FORWARD_KINEMATIC calculate the transformation matrix of a serial chain from
%the base to the end effector
%   alpha, d, theta, r : DH parameters of the chain
%   t_base : translation vector from the base to the first joint
%   t_end : translation vector from the last joint to the end effector
%   rot : rotation vector the last joint to the end effector

%   T : the final transformation matrix
%   T_interm : all the intermediate transformation matrices between joint joint i-1 and i
%   T_chain : transformation matrices between joint i and the base
%T : (0Ti) matrix to transform from joint to base, joint i reltive to frame 0)
%global T_global
%% Calculate the transformation matrix
% create the translation matrix from the base to the first joint
T = makehgtformS('translate',t_base);
% the chain of transformation matrices includes the last effector
%T_chain = zeros(4,4,length(alpha)+1);
%T=T_global*T;
% loop through all the DH parameters
for i = 1:length(alpha)
    % get the transformation matrix of the i-th link
    transform = transformation_matrix(alpha(i),d(i),theta(i),r(i));
    % multiply with the transformation matrix from link i to link i+1
    T = T*transform;
    % concatenate to the kinematic chain
    T_chain(:,:,i) = T;
end
% multiply with the rotation matrices : first z, then x and y
T = T*makehgtformS('zrotate',rot(3));
T = T*makehgtformS('xrotate',rot(1));
T = T*makehgtformS('yrotate',rot(2));
% multiply with the translation matrix from last joint to the end effector
T = T*makehgtformS('translate',t_end);
% multiply with the fix of the rotation around the z-axis
T = T*makehgtformS('zrotate',rot_fix);
T_end = T;
% concatenate to the kinematic chain 
T_chain(:,:,length(alpha)+1) = T;
end