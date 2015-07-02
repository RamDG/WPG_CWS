function [ J ] = Jacobian( T_chain, size_chain )
%JACOBIAN calculate the jacobian matrix of a segment of the robot
%   T_chain : transformation matrices of the kinematic chain wrt the global
%   frame
%   T_interm : intermediate transformations matrices wrt the joint frames
%   size_chain : size of the kinematic chain = number of joints

%   J : jacobian matrix of the kinematic chain

%% Calculation of the jacobian
% position of the last joint
R=T_chain(1:3,1:3,end);
p_end = T_chain(1:3,4,end); %wrt to end effector
%p_end = T_chain(1:3,4,end-1);% wrt to the last joint
% initialisation
J = zeros(6,size_chain);
% loop through all the links of the chain
for i = 1:size_chain
     a = T_chain(1:3,3,i);
    % get the position vector of the i-th joint
    p_i = T_chain(1:3,4,i);
    % calculate the i-th column of the jacobian
    J(:,i) = [skew(a)*(p_end-p_i); a];
end
