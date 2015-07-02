%% Open the csv file and read the datas
position = csvread('posem.csv');

%% Initial position readen from the csv file
HeadYawAngle       = position(1);
HeadPitchAngle     = position(2);

LShoulderPitchAngle = position(3);
LShoulderRollAngle  = position(4);
LElbowYawAngle      = position(5);
LElbowRollAngle     = position(6);
LWristYawAngle      = position(7);
LHandAngle          = position(8);

RShoulderPitchAngle = position(21);
RShoulderRollAngle  = position(22);
RElbowYawAngle      = position(23);
RElbowRollAngle     = position(24);
RWristYawAngle      = position(25);
RHandAngle          = position(26);

LHipYawPitchAngle   = position(9);
LHipRollAngle       = position(10);
LHipPitchAngle      = position(11);
LKneePitchAngle     = position(12);
LAnklePitchAngle    = position(13);
LAnkleRollAngle     = position(14);

RHipYawPitchAngle   = position(15);
RHipRollAngle       = position(16);
RHipPitchAngle      = position(17);
RKneePitchAngle     = position(18);
RAnklePitchAngle    = position(19);
RAnkleRollAngle     = position(20);

theta_head = [HeadYawAngle, HeadPitchAngle]';

theta_arm(:,1,1) = [RShoulderPitchAngle, RShoulderRollAngle, RElbowYawAngle,...
    RElbowRollAngle]';
theta_arm(:,2,1) = [LShoulderPitchAngle, LShoulderRollAngle, LElbowYawAngle,...
    LElbowRollAngle]';

theta_leg(:,1,1) = [RHipYawPitchAngle, RHipRollAngle, RHipPitchAngle,...
    RKneePitchAngle, RAnklePitchAngle, RAnkleRollAngle]';
theta_leg(:,2,1) = [LHipYawPitchAngle, LHipRollAngle, LHipPitchAngle,...
    LKneePitchAngle, LAnklePitchAngle, LAnkleRollAngle]';