function [ P_Si ] = get_sensors_position( T_leg_R, T_leg_L )
%GET_SENSORS_POSITION give the position of the force sensors on the feet of
%the robot
%   P_Fi : position of the center of the feet

%% Global position in the ankle frame
global LFsrFL;
global LFsrFR;
global LFsrRL;
global LFsrRR;
global RFsrFL;
global RFsrFR;
global RFsrRL;
global RFsrRR;

%% Calculate the position wrt to the feet positions
P_Si = zeros(3,2,4);
% right foot
temp = T_leg_R*makehgtform('translate',RFsrFL);
P_Si(:,1,1) = temp(1:3,end);

temp = T_leg_R*makehgtform('translate',RFsrFR);
P_Si(:,1,2) = temp(1:3,end);

temp = T_leg_R*makehgtform('translate',RFsrRR);
P_Si(:,1,3) = temp(1:3,end);

temp = T_leg_R*makehgtform('translate',RFsrRL);
P_Si(:,1,4) = temp(1:3,end);

% left foot
temp = T_leg_L*makehgtform('translate',LFsrFL);
P_Si(:,2,1) = temp(1:3,end);

temp = T_leg_L*makehgtform('translate',LFsrFR);
P_Si(:,2,2) = temp(1:3,end);

temp = T_leg_L*makehgtform('translate',LFsrRR);
P_Si(:,2,3) = temp(1:3,end);

temp = T_leg_L*makehgtform('translate',LFsrRL);
P_Si(:,2,4) = temp(1:3,end);

end

