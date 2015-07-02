%% Nao Parameters v4.0
%clear all;
close all;
clc;

%% DH parameters
global alpha_leg_R;
global d_leg_R;
global theta_leg_R;
global t_offset_leg_R;
global r_leg_R;
global t_base_leg_R;
global t_end_leg_R;
global rot_leg_R;
global rot_fix_leg_R;

global alpha_leg_L;
global d_leg_L;
global theta_leg_L;
global t_offset_leg_L;
global r_leg_L;
global t_base_leg_L;
global t_end_leg_L;
global rot_leg_L;
global rot_fix_leg_L;

global alpha_arm_R;
global d_arm_R;
global theta_arm_R;
global t_offset_arm_R;
global r_arm_R;
global t_base_arm_R;
global t_end_arm_R;
global rot_arm_R;
global rot_fix_arm_R;

global alpha_arm_L;
global d_arm_L;
global theta_arm_L;
global t_offset_arm_L;
global r_arm_L;
global t_base_arm_L;
global t_end_arm_L;
global rot_arm_L;
global rot_fix_arm_L;

global alpha_head;
global d_head;
global theta_head_coeff;
global t_offset_head;
global r_head;
global t_base_head;
global t_end_head;
global rot_head;
global rot_fix_head;

global joint_names;

%% Foot sensors position
global LFsrFL;
global LFsrFR;
global LFsrRL;
global LFsrRR;
global RFsrFL;
global RFsrFR;
global RFsrRL;
global RFsrRR;

%% Mass parameters
global M_torso
global com_torso;

global M_head;
global com_head;

global M_arm_R;
global M_arm_L;
global com_arm_L;
global com_arm_R;

global M_leg_R;
global M_leg_L;
global com_leg_L;
global com_leg_R;

global total_mass;

%% Inertia parameters
global I_leg_R;
global I_leg_L;
global I_arm_R;
global I_arm_L;
global I_head;
global I_torso;

%% Maximum velocity of the joints
global max_v_arm;
global max_v_leg;

%% Joint Limits
global q_lim_leg_R;
global q_lim_leg_L;
global q_lim_arm_R;
global q_lim_arm_L;
global q_lim_head;
global Rankle Lankle
%% Other parameters
global size_arms;
global size_legs;
global size_head;

%% Dimensions parameters
NeckOffsetZ = 126.50*10^-3;
topCameraX = 53.9*10^-3;
topCameraZ = 67.9*10^-3;

ShoulderOffsetY = 98*10^-3;
ElbowOffsetY = 15*10^-3;
ShoulderOffsetZ = 100*10^-3;
UpperArmLength = 105*10^-3;
HandOffsetX = 57.75*10^-3;
LowerArmLength = 55.95*10^-3;

HipOffsetY = 50*10^-3;
HipOffsetZ = 85*10^-3;
ThighLength = 100*10^-3;
TibiaLength = 102.90*10^-3;
FootHeight = 45.19*10^-3;

%% Mass parameters initialisation
% toso parameters
M_torso = 1.04956;
com_torso = [-0.00413; 0.00009; 0.04342];

% head parameters
M_HeadYaw = 0.06442;
M_HeadPitch = 0.60533;

com_HeadYaw = [-0.00001; 0.00014; -0.02742];
com_HeadPitch = [-0.00112; 0.00003; 0.05258];

M_head = [M_HeadYaw, M_HeadPitch];
com_head = [com_HeadYaw, com_HeadPitch];

% arms parameters
M_Shoulder = 0.07504;
M_Biceps_R = 0.15794;
M_Biceps_L = 0.15777;
M_Elbow = 0.06483;
M_ForeArm_R = 0.07778;
M_ForeArm_L = 0.07761;
M_Hand = 0.18533;

com_Shoulder_R = [-0.00165; 0.02663; 0.00014];
com_Shoulder_L = [-0.00165; -0.02663; 0.00014];
com_Biceps_R = [0.02429; -0.00952; 0.00320];
com_Biceps_L = [0.02455; 0.00563; 0.00330];
com_Elbow = [-0.02744; 0.0; -0.00014];
com_ForeArm_R = [0.02552; -0.00281; 0.0009];
com_ForeArm_L = [0.02556; 0.00281; 0.00076];
com_Hand = [0.03434; -0.00088; 0.00308];

% M_arm_R = [M_Shoulder, M_Biceps_R, M_Elbow, M_ForeArm_R];
% M_arm_L = [M_Shoulder, M_Biceps_L, M_Elbow, M_ForeArm_L];
% com_arm_R = [com_Shoulder_R, com_Biceps_R, com_Elbow, com_ForeArm_R];
% com_arm_L = [com_Shoulder_L, com_Biceps_L, com_Elbow, com_ForeArm_L];

%% attempt to consider the mass of the hand
M_arm_R = [M_Shoulder, M_Biceps_R, M_Elbow, (M_ForeArm_R + M_Hand) ];
M_arm_L = [M_Shoulder, M_Biceps_L, M_Elbow, (M_ForeArm_L + M_Hand) ];
com_arm_R = [com_Shoulder_R, com_Biceps_R, com_Elbow, (com_ForeArm_R*M_ForeArm_R + com_Hand*M_Hand)/(M_ForeArm_R + M_Hand) ];
com_arm_L = [com_Shoulder_L, com_Biceps_L, com_Elbow, (com_ForeArm_L*M_ForeArm_L + com_Hand*M_Hand)/(M_ForeArm_L + M_Hand) ];

% legs parameters
M_Pelvis_R = 0.07118;
M_Pelvis_L = 0.06981;
M_Hip = 0.13053;
M_Thigh_R = 0.38976;
M_Thigh_L = 0.38968;
M_Tibia_R = 0.29163;
M_Tibia_L = 0.29142;
M_Ankle_R = 0.13415;
M_Ankle_L = 0.13416;
M_Foot_R = 0.16171;
M_Foot_L = 0.16184;

com_Pelvis_R = [-0.00766; 0.012; 0.02716];
com_Pelvis_L = [-0.00781; -0.01114; 0.02661];
com_Hip_R = [-0.01549; -0.00029; -0.00516];
com_Hip_L = [-0.01549; 0.00029; -0.00515];
com_Thigh_R = [0.00139; -0.00225; -0.05374];
com_Thigh_L = [0.00138; 0.00221; -0.05373];
com_Tibia_R = [0.00394; -0.00221; -0.04938];
com_Tibia_L = [0.00453; 0.00225; -0.04936];
com_Ankle_R = [0.00045; -0.0003; 0.00684];
com_Ankle_L = [0.00045; 0.00029; 0.00685];
com_Foot_R = [0.0254; -0.00332; -0.03239];
com_Foot_L = [0.02542; 0.0033; -0.03239];

M_leg_R = [M_Pelvis_R, M_Hip, M_Thigh_R, M_Tibia_R, M_Ankle_R, M_Foot_R];
M_leg_L = [M_Pelvis_L, M_Hip, M_Thigh_L, M_Tibia_L, M_Ankle_L, M_Foot_L];
com_leg_R = [com_Pelvis_R, com_Hip_R, com_Thigh_R, com_Tibia_R, com_Ankle_R, com_Foot_R];
com_leg_L = [com_Pelvis_L, com_Hip_L, com_Thigh_L, com_Tibia_L, com_Ankle_L, com_Foot_L];

% total mass
total_mass = sum(M_leg_R) + sum(M_leg_L) + sum(M_arm_R) + sum(M_arm_L) + sum(M_head) + M_torso;

%% DH parameters
% DH parameters of the right leg
alpha_leg_R = [-pi/4,-pi/2,pi/2,0,0,-pi/2];
d_leg_R = [0,0,0,0,0,0];
theta_leg_R = [1,1,1,1,1,1];
t_offset_leg_R = [-pi/2,-pi/4,0,0,0,0];
r_leg_R = [0,0,0,-ThighLength,-TibiaLength,0];
t_base_leg_R = [0,-HipOffsetY,-HipOffsetZ];
t_end_leg_R = [0,0,-FootHeight];
rot_leg_R = [0,-pi/2,pi];
rot_fix_leg_R = 0;

% DH parameters of the left leg
alpha_leg_L = [-3*pi/4,-pi/2,pi/2,0,0,-pi/2];
d_leg_L = [0,0,0,0,0,0];
theta_leg_L = [1,1,1,1,1,1];
t_offset_leg_L = [-pi/2,pi/4,0,0,0,0];
r_leg_L = [0,0,0,-ThighLength,-TibiaLength,0];
t_base_leg_L = [0,HipOffsetY,-HipOffsetZ];
t_end_leg_L = [0,0,-FootHeight];
rot_leg_L = [0,-pi/2,pi];
rot_fix_leg_L = 0;

% DH parameters of the right arm
alpha_arm_R = [-pi/2,pi/2,-pi/2,pi/2];
d_arm_R = [0,0,-UpperArmLength,0];
theta_arm_R = [1,1,-1,1];
t_offset_arm_R = [0,pi/2,0,0];
r_arm_R = [0,0,0,0];
t_base_arm_R = [0,-ShoulderOffsetY-ElbowOffsetY,ShoulderOffsetZ];
t_end_arm_R = [-HandOffsetX-LowerArmLength,0,0];
rot_arm_R = [0,0,pi/2];
rot_fix_arm_R = -pi;

% DH parameters of the left arm
alpha_arm_L = [-pi/2,pi/2,-pi/2,pi/2];
d_arm_L = [0,0,UpperArmLength,0];
theta_arm_L = [1,1,1,1];
t_offset_arm_L = [0,-pi/2,0,0];
r_arm_L = [0,0,0,0];
t_base_arm_L = [0,ShoulderOffsetY+ElbowOffsetY,ShoulderOffsetZ];
t_end_arm_L = [HandOffsetX+LowerArmLength,0,0];
rot_arm_L = [0,0,pi/2];
rot_fix_arm_L = 0;

% DH parameters of the head
alpha_head = [0,-pi/2];
d_head = [0,0];
theta_head_coeff = [1,1];
t_offset_head = [0,-pi/2];
r_head = [0,0];
t_base_head = [0,0,NeckOffsetZ];
t_end_head = [topCameraX,0,topCameraZ];
rot_head = [pi/2,pi/2,0];
rot_fix_head = 0;

joints = ...
   ['HeadYaw       ';
    'HeadPitch     ';
    'LShoulderPitch';
    'LShoulderRoll ';
    'LElbowYaw     ';
    'LElbowRoll    ';
    'LWristYaw     ';
    'LHand         ';
    'LHipYawPitch  ';
    'LHipRoll      ';
    'LHipPitch     ';
    'LKneePitch    ';
    'LAnklePitch   ';
    'LAnkleRoll    ';
    'RHipYawPitch  ';    
    'RHipRoll      ';
    'RHipPitch     ';
    'RKneePitch    ';
    'RAnklePitch   ';
    'RAnkleRoll    ';
    'RShoulderPitch';
    'RShoulderRoll ';
    'RElbowYaw     ';
    'RElbowRoll    ';
    'RWristYaw     ';
    'RHand         ';
    'Time          '];
    
    
joint_names = cellstr(joints);

%% Foot sensors position
LFsrFL = [0.07025; 0.0299; 0];
LFsrFR = [0.07025; -0.0231; 0];
LFsrRL = [-0.03025; 0.0299; 0];
LFsrRR = [-0.02965; -0.0191; 0];
RFsrFL = [0.07025; 0.0231; 0];
RFsrFR = [0.07025; -0.0299; 0];
RFsrRL = [-0.03025; 0.0191; 0];
RFsrRR = [-0.02965; -0.0299; 0];

% synchronise the units
% LFsrFL = LFsrFL*10^3;
% LFsrFR = LFsrFR*10^3;
% LFsrRL = LFsrRL*10^3;
% LFsrRR = LFsrRR*10^3;
% RFsrFL = RFsrFL*10^3;
% RFsrFR = RFsrFR*10^3;
% RFsrRL = RFsrRL*10^3;
% RFsrRR = RFsrRR*10^3;


%% Inertia parameters
I_torso = [0.00506234058, 0.00001431158, 0.00015519082;...
    0.00001431158, 0.00488013591, -0.00002707934;...
    0.00015519082, -0.00002707934, 0.00161030006];

I_HeadYaw = [0.00007499295, 0.00000000157, -0.00000001834;...
    0.00000000157, 0.00007599995, -0.00000005295;...
    -0.00000001834, -0.00000005295, 0.00000553373];

I_HeadPitch = [0.00263129518, 0.00000878814, 0.00004098466;...
    0.00000878814, 0.00249112488, -0.00002995792;...
    0.00004098466, -0.00002995792, 0.00098573565];

I_head(:,:,1) = I_HeadYaw;
I_head(:,:,2) = I_HeadPitch;

I_Shoulder = [0.0000842843, 0.00000202802, 0.00000002338;...
    0.00000202802, 0.00001415561, 0.00000001972;...
    0.00000002338, 0.00000001972, 0.00008641949];

I_Biceps_R = [0.00011012031, 0.00007669131, -0.00002604607;...
    0.00007669161, 0.00039757653, 0.00001209828;...
    -0.00002604607, 0.00001209828, 0.00035461772];

I_Biceps_L = [0.00009389993, -0.00004714452, -0.00002699471;...
    -0.00004714452, 0.00037151879, -0.00000245977;...
    -0.00002699471, -0.00000245977, 0.00034190083];

I_Elbow = [0.00000559715, 0.00000000421, 0.00000004319;...
    0.00000000421, 0.00007543312, -0.00000000184;...
    0.00000004319, -0.00000000184, 0.00007644339];

I_ForeArm_R = [0.00002539070, 0.00000233243, -0.00000060117;...
    0.00000233243, 0.00008922036, 0.00000002694;...
    -0.00000060117, 0.00000002694, 0.00008724843];

I_ForeArm_L = [0.00002533220, -0.00000234271, 0.00000007459;...
    -0.00000234271, 0.00008913220, -0.00000002655;...
    0.00000007459, -0.00000002655, 0.00008728726];

I_Hand = [0.00007054933, 0.00000571599, -0.00002247437;...
    0.00000571599, 0.00035606232, 0.00000317771;...
    -0.00002247437, 0.00000317771, 0.00035191933];

I_arm_R(:,:,1) = I_Shoulder;
I_arm_R(:,:,2) = I_Biceps_R;
I_arm_R(:,:,3) = I_Elbow;
I_arm_R(:,:,4) = I_ForeArm_R;
%I_arm_R(:,:,5) = I_Hand;

I_arm_L(:,:,1) = I_Shoulder;
I_arm_L(:,:,2) = I_Biceps_L;
I_arm_L(:,:,3) = I_Elbow;
I_arm_L(:,:,4) = I_ForeArm_L;
%I_arm_L(:,:,5) = I_Hand;

I_Pelvis_R = [0.00008997195, 0.00000500219, 0.00001273525;...
    0.00000500219, 0.00010552611, -0.00002770080;...
    0.00001273525, -0.00002770080, 0.00006688724];

I_Pelvis_L = [0.00008150233, -0.00000499449, 0.00001274817;...
    -0.00000499449, 0.00010132555, 0.00002345474;...
    0.00001274817, 0.00002345474, 0.00006262363];

I_Hip_R = [0.00002758654, -0.00000001919, -0.00000410822;...
    -0.00000001919, 0.00009826996, 0.00000000251;...
    -0.00000410822, 0.00000000251, 0.00008810332];

I_Hip_L = [0.00002758354, -0.00000002233, -0.00000408164;...
    -0.00000002233, 0.00009827055, -0.00000000419;...
    -0.00000408164, -0.00000000419, 0.00008809973];

I_Thigh_R = [0.00163748205, -0.00000083954, 0.00008588301;...
    -0.00000083954, 0.00159221403, -0.00003917626;...
    0.00008588301, -0.00003917626, 0.00030397824];

I_Thigh_L = [0.00163671962, 0.00000092451, 0.00008530668;...
    0.00000092451, 0.00159107278, 0.00003836160;...
    0.00008530668, 0.00003836160, 0.00030374342];

I_Tibia_R = [0.00118282956, -0.00000089650, 0.00002799690;...
    -0.00000089650, 0.00112827851, -0.00002799690;...
    0.00002799690, -0.00003847604, 0.00019145277];

I_Tibia_L = [0.00118207967, 0.00000063362, 0.00003649697;...
    0.00000063362, 0.00112865226, 0.00003949523;...
    0.00003649697, 0.00003949523, 0.00019322744];

I_Ankle_R = [0.00003850813, 0.0000006434, 0.00000387466;...
    0.00000006434, 0.00007431082, -0.00000000458;...
    0.00000387466, -0.00000000458, 0.00005491312];

I_Ankle_L = [0.00003850978, -0.00000002634, 0.00000386194;...
    -0.00000002634, 0.00007426526, 0.00000001834;...
    0.00000386194, 0.00000001834, 0.00005486540];

I_Foot_R = [0.00026930201, 0.00000587505, 0.00013913328;...
    0.00000587505, 0.00064347385, -0.00001884917;...
    0.00013913328, -0.00001884917, 0.00052503477];

I_Foot_L = [0.00026944182, -0.00000569572, 0.00013937948;...
    -0.00000569572, 0.00064434250, 0.00001874092;...
    0.00013937948, 0.00001874092, 0.00052575675];
 
I_leg_R(:,:,1) = I_Pelvis_R;
I_leg_R(:,:,2) = I_Hip_R;
I_leg_R(:,:,3) = I_Thigh_R;
I_leg_R(:,:,4) = I_Tibia_R;
I_leg_R(:,:,5) = I_Ankle_R;
I_leg_R(:,:,6) = I_Foot_R;

I_leg_L(:,:,1) = I_Pelvis_L;
I_leg_L(:,:,2) = I_Hip_L;
I_leg_L(:,:,3) = I_Thigh_L;
I_leg_L(:,:,4) = I_Tibia_L;
I_leg_L(:,:,5) = I_Ankle_L;
I_leg_L(:,:,6) = I_Foot_R;

% synchronise the units
% I_torso = I_torso*10^6;
% I_head = I_head*10^6;
% I_arm_R = I_arm_R*10^6; 
% I_arm_L = I_arm_L*10^6;
% I_leg_R = I_leg_R*10^6; 
% I_leg_L = I_leg_L*10^6;

% total inertia
%total_inertia = I_torso + sum(I_head,3) + sum(I_leg_R,3) + sum(I_leg_L,3) +...
    %sum(I_arm_R,3) + sum(I_arm_L,3);

%% Maximum joint velocity
ShoulderPitchMax = 2.11;
ShoulderRollMax = 1.83;
ElbowYawMax = 2.11;
ElbowRollMax = 1.83;

HipYawMax = 1.65;
HipRollMax = 1.65;
HipPitchMax = 2.54;
KneePitchMax = 2.54;
AnklePitchMax = 2.64;
AnkleRollMax = 1.65;

max_v_arm = [ShoulderPitchMax,ShoulderRollMax,ElbowYawMax,ElbowRollMax]';
max_v_leg = [HipYawMax,HipRollMax,HipPitchMax,KneePitchMax,AnklePitchMax,AnkleRollMax]';

%% Joint Limits [min,max]
HeadYaw = [-2.0857 , 2.0857];
HeadPitch = [-0.6720 , 0.5149];

LShoulderPitch = [-2.0857 , 2.0857];
LShoulderRoll = [-0.3142 , 1.3265];
LElbowYaw = [-2.0857 , 2.0857];
LElbowRoll = [-1.5446 , 0.0349];

HipYaw = [-1.145303 , 0.740810];

LHipRoll = [-0.379472 , 0.790477];
LHipPitch = [-1.535889 , 0.484090];
LKneePitch = [-0.092346 , 2.112528];
LAnklePitch = [-1.189516 , 0.922747];
LAnkleRoll = [-0.397880 , 0.769001];

RShoulderPitch = [-2.0857 , 2.0857];
RShoulderRoll = [-1.3265 , 0.3142];
RElbowYaw = [-2.0857,2.0857];
RElbowRoll = [0.0349 , 1.5446];


RHipRoll = [-0.790477 , 0.379472];
RHipPitch = [-1.535889 , 0.484090];
RKneePitch = [-0.103083 , 2.120198];
RAnklePitch = [-1.186448 , 0.932056];
RAnkleRoll = [-0.768992 , 0.397935];

q_lim_leg_R=[HipYaw;RHipRoll;RHipPitch;RKneePitch;RAnklePitch;RAnkleRoll];
q_lim_leg_L=[HipYaw;LHipRoll;LHipPitch;LKneePitch;LAnklePitch;LAnkleRoll];
q_lim_arm_R=[RShoulderPitch;RShoulderRoll;RElbowYaw;RElbowRoll];
q_lim_arm_L=[LShoulderPitch;LShoulderRoll;LElbowYaw;LElbowRoll];
q_lim_head=[HeadYaw;HeadPitch];
 %% Ankle anti collision limitations [Pitch Value  Roll min  Roll Max]
Rankle(1,:)=[-1.189442 	-0.049916 	0.075049];
Rankle(2,:)=[-0.840027 	-0.179943 	0.169995];
Rankle(3,:)=[-0.700051 	-0.397935 	0.220086];
Rankle(4,:)=[-0.449946 	-0.397935 	0.768992];
Rankle(5,:)=[0.100007 	-0.397935 	0.768992];
Rankle(6,:)= [0.349938 	-0.397935 	0.550477];
Rankle(7,:)=[0.922755 	-0.000000 	0.049916];

Lankle(1,:)=[-1.189442 	-0.075049 	0.049916];
Lankle(2,:)=[-0.840027 	-0.169995 	0.179943];
Lankle(3,:)=[-0.700051 	-0.220086 	0.397935];
Lankle(4,:)=[-0.449946 	-0.768992 	0.397935];
Lankle(5,:)=[0.100007 	-0.768992 	0.397935];
Lankle(6,:)=[0.349938 	-0.550477 	0.397935];
Lankle(7,:)=[0.922755 	-0.049916 	0.000000];


%% Other parameters
size_arms = 4;
size_legs = 6;
size_head = 2;