function [ P_Fi, P_SR, P_SL, xi_Fi, lambda] = generateStep( r, alpha,...
    T, nb_points, swing_foot,T_leg_R,T_leg_L,max_height,mid_height)
%GENERATE_STEP generate a trajectory for the foot over one step
%   r : distance from origine pose to goal
%   alpha : angle (polar coordinates)
%   psi : stearing angle
%   t : time of the step
%   foot : value for the foot (1 for right, 2 for left)

%   P_F : position of the center of the feet
%   P_S : position of the four sensors
%   eps_F : linear and angular velocities

%% Global variables
global dt;
global lambda_coeff;

%% Parameters for the trajectory
time = T/10;
step = nb_points/10;
r=r*2;
ds=fix(.05*nb_points);%half of the points of step in double support to consider in beggining and end
ss=nb_points-2*ds;%points of step in single support
%% Generate the trajectory
% initialisation
P_Fi = zeros(3,2,nb_points);
P_SR = zeros(3,4,nb_points);
P_SL = zeros(3,4,nb_points);
xi_Fi = zeros(6,2,nb_points);


%% Initialise the position
for i = 1:nb_points
    P_Fi(:,1,i) = T_leg_R(1:3,4);
    P_Fi(:,2,i) = T_leg_L(1:3,4);
end
% time vector*
t = linspace(dt,T,nb_points);

%%          SS phase

%% X and y trajectory
%linear trajectory for feet
px=linspace(r*cos(alpha)/ss,r*cos(alpha),ss);
py=linspace(r*sin(alpha)/ss,r*sin(alpha),ss);

% x=[1 ss/16 ss*15/16 ss];
% xx=[0 r*cos(alpha)/ss   (r*cos(alpha)-r*cos(alpha)/ss) r*cos(alpha)];
% cs = spline(x, xx );
% p=1:ss;
% px=ppval(cs,p);
% plot(x,xx,'o',p,ppval(cs,p),'-');

% %% x trajectory
% prev=0;
% x=linspace(pi/(ss+1),pi,ss+1);
% px=zeros(nb_points,1);
% for i=ds+1:nb_points-ds
%      px(i)=integrate(sin(x(i-ds)),prev);
%            prev=px(i);  
% end
%     
%       coef=r*cos(alpha)/px(nb_points-ds);
%        px=coef*px;
% 
% 
% %% y trajectory
% prev=0;
% x=linspace(pi/(ss+1),pi,ss+1);
% py=zeros(nb_points,1);
% for i=ds+1:nb_points-ds
%      py(i)=integrate(sin(x(i-ds)),prev);
%            prev=py(i);  
% end
%     
%       coef=r*sin(alpha)/py(nb_points-ds);
%        py=coef*py;

%% Z trajectory
prev=0;
x=linspace(2*pi/(ss+1),2*pi,ss+1);
pz=zeros(nb_points,1);
for i=ds+1:nb_points-ds
     pz(i)=integrate(sin(x(i-ds)),prev);
           prev=pz(i);  
end
    
      coef=max_height/max(pz);
       pz=coef*pz;
       
P_Fi(1,swing_foot,ds+1:nb_points-ds)=P_Fi(1,swing_foot,1)+px';
P_Fi(2,swing_foot,ds+1:nb_points-ds)=P_Fi(2,swing_foot,1)+py';
P_Fi(3,swing_foot,:)=pz';

 
%%changle last values of x and y 
for i=nb_points+1-ds:nb_points
P_Fi(1,swing_foot,i)=P_Fi(1,swing_foot,nb_points-ds);
P_Fi(2,swing_foot,i)=P_Fi(2,swing_foot,nb_points-ds);
end

%plot(squeeze(P_Fi(1,swing_foot,:))',squeeze(P_Fi(3,swing_foot,:))'); hold on
% figure,
% plot(t,squeeze(P_Fi(3,swing_foot,:))'); hold on
% plot(t,squeeze(P_Fi(1,swing_foot,:))'); hold on



lambda = zeros(8,nb_points); % 4 contacts on each feet

if swing_foot == 1
    lambda(5:8,:) = ones(4,nb_points)*lambda_coeff;
%     % al contacts have force in double support
    lambda(1:4,1:ds) = ones(4,ds)*lambda_coeff;
    lambda(1:4,nb_points+1-ds:nb_points) = ones(4,ds)*lambda_coeff;
else
    lambda(1:4,:) = ones(4,nb_points)*lambda_coeff;
%     % al contacts have force in double support
    lambda(5:8,1:ds) = ones(4,ds)*lambda_coeff;
    lambda(5:8,nb_points+1-ds:nb_points) = ones(4,ds)*lambda_coeff;
end

% %increase load to the back of the feet
for l=1:nb_points
lambda(:,l)=lambda(:,l).*[1;1;1.5;1.5;1;1;1.5;1.5];
end
% %

%% Generate the velocity of the feet
% use the central difference to calculate the velocity
for i = 2:(nb_points-1)
    xi_Fi(1:3,:,i) = (P_Fi(:,:,i+1) - P_Fi(:,:,i-1))/(2*dt);
end

% figure,
%    plot(squeeze( xi_Fi(1,swing_foot,:)),'r'); hold on;
%     plot(squeeze( xi_Fi(2,swing_foot,:))); hold on;
%     plot(squeeze( xi_Fi(3,swing_foot,:)),'g'); hold on;
%     legend('xi_{Fx}','xi_{Fy}','xi_{Fz}',3);
%       xlabel('1 step');
%       ylabel('Linear Velocity, m/s')


% get the position of the foot sensors
for i = 1:nb_points
    % create the transformation matrix with the position of the foot
    T_R = makehgtform('translate',P_Fi(:,1,i));
    T_L = makehgtform('translate',P_Fi(:,2,i));
    temp_ps = get_sensors_position(T_R, T_L);
    P_SR(:,:,i) = temp_ps(:,1,:);
    P_SL(:,:,i) = temp_ps(:,2,:);
end

%% Creation of the vertices
vertix = [P_SR, P_SL];

end
