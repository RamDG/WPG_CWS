function [z_axis]=getAxisFromTwist(xi,R)
%the angular velocites considered is rads per sec
global dt
uthetaB = vrrotmat2vec(R);
cst = [uthetaB(1); uthetaB(2); uthetaB(3)]*uthetaB(4);
gamma=xi(4:6)*dt+cst;
T =makehgtformS('xrotate',gamma(1));
T = T*makehgtformS('yrotate',gamma(2));
T = T*makehgtformS('zrotate',gamma(3));
z_axis=T(1:3,3);

vrrotvec2mat(uthetaB)
end

% uthetaB = vrrotmat2vec(R);
% cst = [uthetaB(1); uthetaB(2); uthetaB(3)]*uthetaB(4);
% gamma=xi(4:6)*dt+cst;
% T =makehgtformS('xrotate',gamma(1));
% T = T*makehgtformS('yrotate',gamma(2));
% T = T*makehgtformS('zrotate',gamma(3));
% z_axis=T(1:3,3);
% 
% vrrotvec2mat(uthetaB)
% 
% gamma=[pi,pi/2,pi/4]
% 
% uthetaB=[1,0,0,-pi/2]
% 
% uth=vrrotmat2vec(T(1:3,1:3))
% cst = [uth(1); uth(2); uth(3)]*uth(4);
% 
% 
% q=angle2quat(gamma(1),gamma(2),gamma(3),'XYZ')
% ut=[q(2)/sqrt(1-q(1)^2),q(3)/sqrt(1-q(1)^2),q(4)/sqrt(1-q(1)^2),2*acos(q(1))]
% 
% vrrotvec2mat(ut)