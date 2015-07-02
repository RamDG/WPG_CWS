%Script to create symbolic jacobians of end effectors and com to apply
%following task : given end effectors positions move the com to desired
%position
 clc;
close all;





run('init_nao_parameters.m');

%% Global variables
global steptime;
global step;
global epsilon;
global dt;
global Lk0;
global Lk1;
global dx_G0;
global dy_G0;
global dz_G0;
global g;
global z0;
global T_leg_R;
global T_leg_L;
global joint_names;

global chain_leg_R;
global chain_leg_L;
global chain_arm_R;
global chain_arm_L;
global chain_head;

global P_SR;
global P_SL;
global P_B_COG;
global zG;
global dP_B_COG;
global com_measured;
global com_desired;

global start;
global limit_vel;

global max_height;
global mid_height;
global lambda_coeff;
global total_mass

global Alpha;
global Kp;
global Ki;
global Kd;

global P_B_Fi;
global P_B_Hi;
global Mt;
global Ht;
global J_arm_inv;
global J_leg_inv;
global total_inertia;

global T_global;
%%Initialisation of global reference transformation matrix
T_global=eye(4);


%% Initialisation of the gravity constant
g = 9.81;

%% Initialisation of parameters for the PID
Alpha = 1;
Ki = 5;

%% Initiliastion of epsilon, threshold to stop the loop
epsilon = 0.0001;

%% Parameter for criterion
alpha=0;

%% Initialisation of the angular momentum of the robot
Lk0 = zeros(3,1); % not to be modified
Lk1 = zeros(3,1); % not to be modified

%% Parameters of the trajectory
nb_points = 20;
nb_steps = 20;
swing_foot = 1; % not to be modified
step_time = 1;
step_length = 50*10^-3;
max_height = 10*10^-3;
mid_height = 2*10^-3;
lambda_coeff = 0.2;



%% Initialisation of the time step
dt = step_time/nb_points; % not to be modified

%% Initialisation of the time for the animation
steptime = dt; % not to be modified

%% Initialisation of the reference velocities
%TODO: eps_ variables should be called xi_ double check the variable in the
%theory (not so important but easier to compare with theory)
% velocity of both feet
eps_Fi_traj = zeros(6,2,nb_points);
% velocity of both hands
eps_Hi_traj = zeros(6,2,nb_points);
% velocity of the waist
eps_B_traj = zeros(6,1,nb_points);
% velocity of the COG
dx_G0 = 0;
dy_G0 = 0;
dz_G0 = 0;

%% Initialisation of the accelerations
ddz_G_traj = zeros(1,1,nb_points);

%% Initialisation of the starting position
theta_arm = zeros(4,2,nb_points*nb_steps);
theta_leg = zeros(6,2,nb_points*nb_steps);

run('init_robot_position.m');

t_arm_0 = theta_arm(:,:,1);
t_leg_0 = theta_leg(:,:,1);

%% Parameters for the simulation
com_measured = zeros(3,nb_points*nb_steps);
start = 1; % not to be modified
step = 1; % not to be modified
traj_time = zeros(1,nb_points*nb_steps);

pid_vel = 0;
limit_vel = 1;
plot_res = 1;
animation = 1;


RL1=sym('RL1'); RL2=sym('RL2'); RL3=sym('RL3'); RL4=sym('RL4'); RL5=sym('RL5'); RL6=sym('RL6');
LL1=sym('LL1'); LL2=sym('LL2'); LL3=sym('LL3'); LL4=sym('LL4'); LL5=sym('LL5'); LL6=sym('LL6');
RA1=sym('RA1'); RA2=sym('RA2'); RA3=sym('RA3'); RA4=sym('RA4');
LA1=sym('LA1'); LA2=sym('LA2'); LA3=sym('LA3'); LA4=sym('LA4');
H1=sym('H1'); H2=sym('H2');
    
RL=[RL1; RL2; RL3; RL4; RL5; RL6];
RL = sym(RL);

LL=[ LL1; LL2; LL3; LL4; LL5; LL6 ];
LL = sym(LL);

RA= [ RA1; RA2; RA3; RA4 ];
RA = sym(RA);

LA= [ LA1; LA2; LA3; LA4 ];
LA = sym(LA);

H= [ H1; H2 ];
H = sym (H);

qAll= [ RL1; RL2; RL3; RL4; RL5; RL6; LL1; LL2; LL3; LL4; LL5; LL6; RA1; RA2; RA3; RA4; LA1; LA2; LA3; LA4; H1; H2 ];
qAll= sym (qAll);
% theta_leg,theta_arm,theta_head where the initial coordinates are stored
theta_All= [ theta_leg(:,1,1); theta_leg(:,2,1); theta_arm(:,1,1); theta_arm(:,2,1); theta_head ];
%%%%%%%%%%%% double values
% 
% [T_leg_R, chain_leg_R] = forward_chain('leg_R',theta_leg(:,1)');
% [T_leg_L, chain_leg_L] = forward_chain('leg_L',theta_leg(:,2)');
% 
% [T_arm_R, chain_arm_R] = forward_chain('arm_R',theta_arm(:,1)');
% [T_arm_L, chain_arm_L] = forward_chain('arm_L',theta_arm(:,2)');
% 
% [~, chain_head] = forward_chain('head',theta_head');
%  [ chain_leg_R, chain_leg_L, chain_arm_R,chain_arm_L, chain_head]...
%        = global_reference( chain_leg_R, chain_leg_L, chain_arm_R,...
%         chain_arm_L, chain_head,swing,s);
% % deEff = [ T_leg_R(1:3,4) ;  T_leg_R(1:3,3) ; T_leg_L(1:3,4) ;T_leg_L(1:3,3) ; T_arm_R(1:3,4) ;T_arm_R(1:3,3) ; T_arm_L(1:3,4) ; T_arm_L(1:3,3) ; chain_head(1:3,4,2) ; chain_head(1:3,3,2)];
% deEff = [ chain_leg_R(1:3,4,end) ;  chain_leg_R(1:3,3,end) ; chain_leg_L(1:3,4,end) ;chain_leg_L(1:3,3,end)];

%%%%%%%%%% symbolic
RL=RL'
[T_leg_R2, chain_leg_R] = forward_chain('leg_R',RL');
[T_leg_L2, chain_leg_L] = forward_chain('leg_L',LL');

[T_arm_R2, chain_arm_R] = forward_chain('arm_R',RA');
[T_arm_L2, chain_arm_L] = forward_chain('arm_L',LA');

[~, chain_head] = forward_chain('head',H');

%% Position of the feet and hands NOTE: might be relevant to take pose not position
%P_B_Fi = [T_leg_R(1:3,4),T_leg_L(1:3,4)];
%P_B_Hi = [T_arm_R(1:3,4),T_arm_L(1:3,4)];

% matlabFunction(chain_leg_R, chain_leg_L,chain_arm_R,chain_arm_L,chain_head, 'File', 'chains',...
%'Vars', {RL,LL,RA,LA,H},'Outputs', {'chain_leg_R','chain_leg_L','chain_arm_R','chain_arm_L','chain_head'});

[chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head]=global_reference(chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head);
    %% Calculation of the COM
    P_B_COG2 = center_of_gravity( chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head );

J_RFoot=jacobian(chain_leg_R(1:3,4,:),RL);
   % J_RFoot=jacobian(T_leg_R(1:3,4),RL);
   % J_RFoot=jacobian(T_leg_R(1:3,4),qAll);
J_COM=jacobian(P_B_COG2 , qAll);

COG= subs(P_B_COG2,qAll,theta_All);
j_COM=double(vpa( subs(J_COM,qAll,theta_All),5));


dCOG=[0.0093406026;0.000011748754;0.22148217];
% for i=1:size(chain_leg_R2,3)
% vpa(subs(chain_leg_R2(:,:,i), RL, theta_leg(:,1,1)),4)
% end
% vpa(subs(T_leg_L2, LL, theta_leg(:,2,1)),4)
% vpa(subs(T_arm_R2, RA, theta_arm(:,1,1)),4)
% vpa(subs(T_arm_L2, LA, theta_arm(:,2,1)),4)

% 
% for i=1:size(chain_arm_R2,3)
% vpa(subs(chain_arm_R2(:,:,i), RA, theta_arm(:,1,1)),4)
%  end
% 
% vpa(T_leg_R2(1:3,3),5);
% J_RFoot=vpa(jacobian(T_leg_R2(1:3,4),RL),5);
% 
% 
% vpa(subs(J_RFoot, RL, theta_leg(:,1,1)),4)



%%%%%%%%%%% Formula to get q
% eEff = [ T_leg_R(1:3,4) ;  T_leg_R(1:3,3) ; T_leg_L(1:3,4) ;T_leg_L(1:3,3) ; T_arm_R(1:3,4) ;T_arm_R(1:3,3) ; T_arm_L(1:3,4) ; T_arm_L(1:3,3) ; chain_head(1:3,4,2) ; chain_head(1:3,3,2)];
%eEff = [ T_leg_R(1:3,4) ;  T_leg_R(1:3,3) ; T_leg_L(1:3,4) ;T_leg_L(1:3,3)];
eEff=deEff;
dx1=deEff-eEff;
dx2=(dCOG-double(vpa(COG,5)));
iter=0;
ok=false;
%%%% iteratively find the values
while ( iter<1000 && ~ok)
    %%%%%%%%%%%%%% calculate chains
[T_leg_R, chain_leg_R] = forward_chain('leg_R',theta_leg(:,1)');
[T_leg_L, chain_leg_L] = forward_chain('leg_L',theta_leg(:,2)');

[T_arm_R, chain_arm_R] = forward_chain('arm_R',theta_arm(:,1)');
[T_arm_L, chain_arm_L] = forward_chain('arm_L',theta_arm(:,2)');

[~, chain_head] = forward_chain('head',theta_head');
[chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head]=global_reference(chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head);
    %%%%%%%%%%%%%%%%%%% Jacobian of end effectors
J_leg_R = Jacobian(chain_leg_R, size(theta_leg,1));
J_leg_L = Jacobian(chain_leg_L, size(theta_leg,1));

J_arm_R = Jacobian(chain_arm_R, size(theta_arm,1));
J_arm_L = Jacobian(chain_arm_L, size(theta_arm,1));

J_head = Jacobian(chain_head, size(theta_head,1));

theta_All= [ theta_leg(:,1,1); theta_leg(:,2,1); theta_arm(:,1,1); theta_arm(:,2,1); theta_head ];
j_COM=double(vpa( subs(J_COM,qAll,theta_All),5));

zerL=zeros(size(J_leg_R));
zerA=zeros(size(J_arm_R));
zerH=zeros(6,2);

% J1=[J_leg_R,zerL,zerA,zerA,zerH;
%     zerL,J_leg_L,zerA,zerA,zerH;
%     zerL,zerL,J_arm_R,zerA,zerH;
%     zerL,zerL,zerA,J_arm_L,zerH;
%     zerL,zerL,zerA,zerA,J_head;];

J1=[J_leg_R,zerL,zerA,zerA,zerH;
    zerL,J_leg_L,zerA,zerA,zerH;];
invJ1=inv_jacobian(J1);
P1=invJ1*J1;
P1=eye(size(P1)) - P1;
%%%%%%%%%%%%%%%%%%%%% Jacobian of COG
J2a=[J1;j_COM ];
invJ2a=inv_jacobian(J2a);
P2=invJ2a*J2a;
P2=eye(size(P2)) - P2;

inv_j2p2=inv_jacobian(j_COM*P1);
    
dq= invJ1*dx1 + P1*(inv_jacobian(j_COM))* (dx2 -j_COM* invJ1*dx1 );

%baerlocher et al
dq= invJ1*dx1 + inv_j2p2* (dx2 -j_COM* invJ1*dx1 );
qq=theta_All+.1*dq;

%qq=inv_jacobian(j_COM)*dx2
theta_leg=[qq(1:6),qq(7:12)];
theta_arm=[qq(13:16),qq(17:20)];
theta_head=qq(21:22);
% 

% 
% [chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
%     move_robot(theta_leg2,theta_arm2,theta_head2);
% animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
% eEff = [ T_leg_R(1:3,4) ;  T_leg_R(1:3,3) ; T_leg_L(1:3,4) ;T_leg_L(1:3,3) ; T_arm_R(1:3,4) ;T_arm_R(1:3,3) ; T_arm_L(1:3,4) ; T_arm_L(1:3,3) ; chain_head(1:3,4,2) ; chain_head(1:3,3,2)];
eEff = [ chain_leg_R(1:3,4,end) ;  chain_leg_R(1:3,3,end) ; chain_leg_L(1:3,4,end) ;chain_leg_L(1:3,3,end)];

%P_B_COG = double (vpa( subs(P_B_COG2,qAll,qq),5))
P_B_COG = center_of_gravity( chain_leg_R, chain_leg_L, chain_arm_R,...
        chain_arm_L, chain_head );
%dCOG
dx2 = (dCOG-P_B_COG)
dx1 = deEff-eEff;
sum(abs(dx2))
diff1=sum(abs(dx1))
if (sum(abs(dx2))<.00002 && diff1<.00002)
    ok=true;
end
iter=iter+1;
animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);
end

[chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head] = ...
    move_robot(theta_leg,theta_arm,theta_head);
animate(chain_leg_R, chain_leg_L, chain_arm_R, chain_arm_L, chain_head);