function [J_O]=generateOrientationJacobian(T,j,ant,nb_joints,joint_mapping)
% LFootJ=[JacobianLFoot;O_LFoot];
% LFootJ=sym(LFootJ);

% Order antecedents
a = j;
%by default all the positions to 0 this will ensure that the not related q
%values are set to 0
if isa(T,'sym')
           J_O=sym(zeros(3,nb_joints));
else
J_O=zeros(3,nb_joints);
end
while a ~= 0 %keep until you reach the base
     l=abs(joint_mapping(a));
    si=sign(joint_mapping(a));
    if l~=0
    J_O(:,l)= J_O(:,l)+si*T(1:3,3,a);%take the z axis of the transformation matrix
    end
    a=ant(a);
  
end

%  C = [0 1 0 1; 1 0 1 0; 0 0 1 1];      % The sample input matrix
%  Q = sym(sym('Q%d%d',size(C)),'real')  % Create a matrix of Qxy variables

 %Y = sum(sym(C).*Q,2)  % Perform element-wise multiplication of Q by a
                         %   symbolic version of C, then sum the result
end

