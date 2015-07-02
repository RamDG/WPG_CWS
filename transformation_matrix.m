function [ T ] = transformation_matrix( alpha, d, theta, r )
%TRANSFORMATION_MATRIX calculate the transformation matrix with the input
%DH parameters NOTE: the operation order seems to be modified DH
%   alpha : rotation around x axis
%   theta : rotation around z axis
%   d : translation around z axis
%   r : translation around x axis

%   T : the transformation matrix

%% Calculate the transformation matrix
%T = makehgtform('xrotate',alpha,'translate',[r,0,0],'zrotate',theta,'translate',[0,0,d]);%Original
T = makehgtformS('xrotate',alpha,'translate',[r,0,0],'zrotate',theta,'translate',[0,0,d]);%to use theta as symbolic
end