

% NAO H25

clear
clc

% PARAMETERS

SaveFig = false;

% 'plot'
% 'symbolic'
symbolic = 'symbolic';




% For symbolic development
    q1=sym('q1'); q2=sym('q2'); q3=sym('q3'); q4=sym('q4'); q5=sym('q5'); q6=sym('q6');
    q7=sym('q7'); q8=sym('q8'); q9=sym('q9'); q10=sym('q10'); q11=sym('q11');
    q12=sym('q12'); q13=sym('q13'); q14=sym('q14'); q15=sym('q15');
    q16=sym('q16'); q17=sym('q17'); q18=sym('q18'); q19=sym('q19');
    q20=sym('q20'); q21=sym('q21'); q22=sym('q22'); q23=sym('q23');
    q=[q1;q2;q3;q4;q5;q6;q7;q8;q9;q10;q11;q12;q13;q14;q15;q16;q17;q18;q19;q20;q21;q22;q23];
    q = sym(q);
global T_transl
T_transl=eye(4);

%%
% File to load the parameters
PARAMETERS_RFoot;
matlabFunction(T, 'File', 'RTreeStructure',...
'Vars', {q},'Outputs', {'T'});
%   % DIRECT KINEMATICS -- SYMBOLIC support Right Leg
%   %Mapping relates which joint is at the position related to the
%   %antecedents "a"
%     mapping = [1,-2,-3,-4,-5,-6,0,6,7,8,9,10,11,...
%     0,12,13,14,15,16,0,17,18,19,20,21,0,22,23];
%     % Left Foot sole
%     k = 14;
%     P_LFoot = T(1:3,4,k);
%     P_LFoot = vpa(P_LFoot,4); % Represent as decimals instead of fractions
%    
%     %Jacobian
%     JacobianLFoot =jacobian(P_LFoot,q);
%     JacobianLFoot = vpa(JacobianLFoot,4);
%     JO_LFoot=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%      J_LFoot=[JacobianLFoot;JO_LFoot];
%     
%     % Right Hand
%     k = 20;
%     P_RHand = T(1:3,4,k);
%     P_RHand = vpa(P_RHand,4); % Represent as decimals instead of fractions
%     
%     %Jacobian
%     JacobianRHand=jacobian(P_RHand,q);
%     JacobianRHand = vpa(JacobianRHand,4);
%     JO_RHand=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%     J_RHand=[JacobianRHand;JO_RHand];
%      
%     % Left Hand
%     k = 26;
%     P_LHand = T(1:3,4,k);
%     P_LHand = vpa(P_LHand,4); % Represent as decimals instead of fractions
%    
%      %Jacobian
%     JacobianLHand=jacobian(P_LHand,q);
%     JacobianLHand = vpa(JacobianLHand,4);
%    JO_LHand=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%      J_LHand=[JacobianLHand;JO_LHand];
%     % Torso
%     k = 7;
%     P_Torso = T(1:3,4,k);
%     P_Torso = vpa(P_Torso,4); % Represent as decimals instead of fractions
%    
%    %Jacobian
%     JacobianTorso=jacobian(P_Torso,q);
%     JacobianTorso = vpa(JacobianTorso,4);
%     JO_Torso=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%     J_Torso=[JacobianTorso;JO_Torso];
%     
% %     %CoM[x,y,z]
% %     P_CoM = transpose(CoM);
% %     P_CoM = vpa(P_CoM,4);
% %     
% %       % JACOBIAN    
% %     JacobianCoM=jacobian(P_CoM,q);
% %     JacobianCoM = vpa(JacobianCoM,4);
%     
% 
% matlabFunction(J_LFoot,J_Torso, J_RHand,J_LHand, 'File', 'RightJacobians',...
% 'Vars', {q},'Outputs', {'J_LFoot','J_Torso','J_RHand','J_LHand'});
% 
% %% Left Leg
% % File to load the parameters
PARAMETERS_LFoot;
matlabFunction(T, 'File', 'LTreeStructure',...
'Vars', {q},'Outputs', {'T'});
%  % DIRECT KINEMATICS -- SYMBOLIC support on left foot
%  %Mapping relates which joint is at the position related to the
%   %antecedents "a"
%     mapping = [-1,-2,-3,-4,-5,6,0,-6,7, 8, 9, 10,11,...
%     0,12,13,14,15,16,0,17,18,19,20,21,0,22,23];
%     % Right Foot sole
%     k = 14;
%     P_RFoot = T(1:3,4,k);
%     P_RFoot = vpa(P_RFoot,4); % Represent as decimals instead of fractions
%     
%      %Jacobian
%     JacobianRFoot =jacobian(P_RFoot,q);
%     JacobianRFoot = vpa(JacobianRFoot,4);
%     JO_RFoot=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%     J_RFoot=[JacobianRFoot;JO_RFoot];
% % Right Hand
%     k = 20;
%     P_RHand = T(1:3,4,k);
%     P_RHand = vpa(P_RHand,4); % Represent as decimals instead of fractions
%     
%     %Jacobian
%     JacobianRHand=jacobian(P_RHand,q);
%     JacobianRHand = vpa(JacobianRHand,4);
%     JO_RHand=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%     J_RHand=[JacobianRHand;JO_RHand];
%      
%     % Left Hand
%     k = 26;
%     P_LHand = T(1:3,4,k);
%     P_LHand = vpa(P_LHand,4); % Represent as decimals instead of fractions
%    
%      %Jacobian
%     JacobianLHand=jacobian(P_LHand,q);
%     JacobianLHand = vpa(JacobianLHand,4);
%    JO_LHand=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%      J_LHand=[JacobianLHand;JO_LHand];
%     % Torso
%     k = 7;
%     P_Torso = T(1:3,4,k);
%     P_Torso = vpa(P_Torso,4); % Represent as decimals instead of fractions
%    
%    %Jacobian
%     JacobianTorso=jacobian(P_Torso,q);
%     JacobianTorso = vpa(JacobianTorso,4);
%     JO_Torso=generateOrientationJacobian(T,k,a,size(q,1),mapping);
%     J_Torso=[JacobianTorso;JO_Torso];
%     
% %     %CoM[x,y,z]
% %     P_CoM = transpose(CoM);
% %     P_CoM = vpa(P_CoM,4);
% %     
% % %     % JACOBIAN    
% %     JacobianCoM=jacobian(P_CoM,q);
% %     JacobianCoM = vpa(JacobianCoM,4);
%     
%     matlabFunction(J_RFoot,J_Torso, J_RHand,J_LHand, 'File', 'LeftJacobians',...
% 'Vars', {q},'Outputs', {'J_RFoot','J_Torso','J_RHand','J_LHand'});