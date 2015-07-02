function []=writeOutputFile(theta_arm,theta_leg)
global Ts dt

%% Write the trajectory of the joints as a csv file
%original time line from the algorithm
lt=0:dt:Ts;
% time line changed to fit the response time of nao, when communicating
% through wifi response time is approx 30ms=.03
lt2=0:.03:Ts; %%

%interpolate values to match new timeline

th_leg=zeros(size(theta_leg,1),size(theta_leg,2),size( lt2 , 2 ));
for i=1:size(theta_leg,1)
th_leg(i,1,:)=interp1(lt,squeeze(theta_leg(i,1,:)),lt2);
th_leg(i,2,:)=interp1(lt,squeeze(theta_leg(i,2,:)),lt2);
end
theta_leg=th_leg;

th_arm=zeros(size(theta_arm,1),size(theta_arm,2),size( lt2 , 2 ));
for i=1:size(theta_arm,1)
th_arm(i,1,:)=interp1(lt,squeeze(theta_arm(i,1,:)),lt2);
th_arm(i,2,:)=interp1(lt,squeeze(theta_arm(i,2,:)),lt2);
end
theta_arm=th_arm;
% first combine leg and arm trajectories starting with the leg
% theta = zeros(nb_points*nb_steps,length(joint_names));
theta = zeros(size( lt2 , 2 ),24);

% the size of the right leg is one less than the left one as the hip joint
% is the same
theta_leg_R = zeros(size(theta_leg,1)-1,size( lt2 , 2 ));
theta_leg_L = zeros(size(theta_leg,1),size( lt2 , 2 ));
theta_arm_R = zeros(size(theta_arm,1),size( lt2 , 2 ));
theta_arm_L = zeros(size(theta_arm,1),size( lt2 , 2 ));

% add zeros values for the head and the hands angles
theta_head = zeros(2,size( lt2 , 2 ));
theta_hand_R = zeros(2,size( lt2 , 2 ));
theta_hand_L = zeros(2,size( lt2 , 2 ));

% remove the first value of the right leg
theta_leg_R(:,:) = theta_leg(2:end,1,:);
theta_leg_L(:,:) = theta_leg(:,2,:);

theta_arm_R(:,:) = theta_arm(:,1,:);
theta_arm_L(:,:) = theta_arm(:,2,:);

% theta(:,:) = [theta_head',...
%     theta_arm_L',theta_hand_L',...
%     zeros(size( lt2 , 2 ),1),theta_leg_L(2:end,:)',...
%     zeros(size( lt2 , 2 ),1),theta_leg_R',...
%     theta_arm_R',theta_hand_R',traj_time'];
%
% csvwrite_with_headers('trajectory.csv',theta,joint_names);

% theta(:,:) = [zeros(size( lt2 , 2 ),1),theta_leg_R',...
%     zeros(size( lt2 , 2 ),1),theta_leg_L(2:end,:)',...
%     theta_arm_R',zeros(size( lt2 , 2 ),1),...
%     theta_arm_L',zeros(size( lt2 , 2 ),1),...
%     theta_head'];

theta(:,:) = [zeros(size( lt2 , 2 ),1),theta_leg_R',...
    theta_leg_L(:,:)',...
    theta_arm_R',zeros(size( lt2 , 2 ),1),...
    theta_arm_L',zeros(size( lt2 , 2 ),1),...
    theta_head'];

theta = filtfilt(ones(1,5)/5,1,theta);
csvwrite('trajectory.csv',theta');

