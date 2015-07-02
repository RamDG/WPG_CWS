%function []=readRealRobotJointValues(fileName)
jointV=csvread('/results/JointValues_1_5s_forcesNearAnkle.csv',1,0);
 sentV=csvread('/results/trajectory_1_5s_forcesNearAnkle.csv');
 
 
vel=jointV(2:end,2:end)-jointV(1:end-1,2:end);
testv=sum(vel')';
initV=99999999;
endV=0;
for i=1:size(vel)
   if(testv(i)~=0)
       if(initV==99999999)
       initV=i;
   end
       endV=i;
   
   end
end

relevant=jointV(initV:endV,:);
t=relevant(:,1)-relevant(1,1);
% order=['Time';    1
%     'RHipRoll ';  2
%     'LKneePitch ';3
%     'LHipPitch '; 4
%     ' RKneePitch ';5
%     'LAnklePitch ';6
%     'LHipYawPitch ';7
%     'RAnkleRoll ';8
%     ' RHipPitch ';9
%     ' LHipRoll '; 10
%     'LAnkleRoll ';11
%     ' RAnklePitch']12
orderRL=[7,2,9,5,12,8];
orderLL=[7,10,4,3,6,11];
rightLeg=relevant(:,orderRL)';
leftLeg=relevant(:,orderLL)';
%plot(t,relevant(:,2))


  % Joints
  %Right Leg
      figure,
      plot(t,squeeze(rightLeg(1,:)),'r'); hold on;
      plot(t,squeeze(rightLeg(2,:)),'b'); hold on;
      plot(t,squeeze(rightLeg(3,:)),'g'); hold on;
      plot(t,squeeze(rightLeg(4,:)),'c'); hold on;
      plot(t,squeeze(rightLeg(5,:)),'m'); hold on;
      plot(t,squeeze(rightLeg(6,:)),'k'); hold on;
      
        legend('HipYawPitch','HipRoll ','HipPitch ','KneePitch ','AnklePitch ','AnkleRoll ',6);
      xlabel('Time [sec]');
      ylabel('Joint Values, radians')
%     Left Leg  
       figure,
      plot(t,squeeze(leftLeg(1,:)),'r'); hold on;
      plot(t,squeeze(leftLeg(2,:)),'b'); hold on;
      plot(t,squeeze(leftLeg(3,:)),'g'); hold on;
      plot(t,squeeze(leftLeg(4,:)),'c'); hold on;
      plot(t,squeeze(leftLeg(5,:)),'m'); hold on;
      plot(t,squeeze(leftLeg(6,:)),'k'); hold on;
      
%       load thLeg
%         % Joints
%       figure,
%       plot(squeeze(theta_leg(1,1,:)),'r'); hold on;
%       plot(squeeze(theta_leg(2,1,:)),'b'); hold on;
%       plot(squeeze(theta_leg(3,1,:)),'g'); hold on;
%       plot(squeeze(theta_leg(4,1,:)),'c'); hold on;
%       plot(squeeze(theta_leg(5,1,:)),'m'); hold on;
%       plot(squeeze(theta_leg(6,1,:)),'k'); hold on;
%       
%       figure,
%       plot(squeeze(theta_leg(1,2,:)),'r'); hold on;
%       plot(squeeze(theta_leg(2,2,:)),'b'); hold on;
%       plot(squeeze(theta_leg(3,2,:)),'g'); hold on;
%       plot(squeeze(theta_leg(4,2,:)),'c'); hold on;
%       plot(squeeze(theta_leg(5,2,:)),'m'); hold on;
%       plot(squeeze(theta_leg(6,2,:)),'k'); hold on;

      
     
      tt=0:.03:7.5;
         % Joints
      figure,
      plot(tt,squeeze(sentV(7,:)),'r'); hold on;
      plot(tt,squeeze(sentV(2,:)),'b'); hold on;
      plot(tt,squeeze(sentV(3,:)),'g'); hold on;
      plot(tt,squeeze(sentV(4,:)),'c'); hold on;
      plot(tt,squeeze(sentV(5,:)),'m'); hold on;
      plot(tt,squeeze(sentV(6,:)),'k'); hold on;
        legend('HipYawPitch','Desired ',6);
      xlabel('Time [sec]');
      ylabel('Joint Values, radians')
      
      figure,
      plot(tt,squeeze(sentV(7,:)),'r'); hold on;
      plot(tt,squeeze(sentV(8,:)),'b'); hold on;
      plot(tt,squeeze(sentV(9,:)),'g'); hold on;
      plot(tt,squeeze(sentV(10,:)),'c'); hold on;
      plot(tt,squeeze(sentV(11,:)),'m'); hold on;
      plot(tt,squeeze(sentV(12,:)),'k'); hold on;
      
      %Right leg comparison
      figure,
      plot(t,squeeze(rightLeg(1,:)),'r'); hold on;
      plot(tt,squeeze(sentV(7,:)),'b'); hold on;
      plot(t,squeeze(rightLeg(2,:)),'r'); hold on;
      plot(t,squeeze(rightLeg(3,:)),'r'); hold on;
      plot(t,squeeze(rightLeg(4,:)),'r'); hold on;
      plot(t,squeeze(rightLeg(5,:)),'r'); hold on;
      plot(t,squeeze(rightLeg(6,:)),'r'); hold on;
      plot(tt,squeeze(sentV(2,:)),'b'); hold on;
      plot(tt,squeeze(sentV(3,:)),'b'); hold on;
      plot(tt,squeeze(sentV(4,:)),'b'); hold on;
      plot(tt,squeeze(sentV(5,:)),'b'); hold on;
      plot(tt,squeeze(sentV(6,:)),'b'); hold on;
      legend('Real Robot','Desired ',2);
      xlabel('Time [sec]');
      ylabel('Joint Values, radians')
      
      %Left leg comparison
      figure,
      plot(t,squeeze(leftLeg(1,:)),'r'); hold on;
      plot(tt,squeeze(sentV(7,:)),'b'); hold on;
      plot(t,squeeze(leftLeg(2,:)),'r'); hold on;
      plot(t,squeeze(leftLeg(3,:)),'r'); hold on;
      plot(t,squeeze(leftLeg(4,:)),'r'); hold on;
      plot(t,squeeze(leftLeg(5,:)),'r'); hold on;
      plot(t,squeeze(leftLeg(6,:)),'r'); hold on;
      plot(tt,squeeze(sentV(8,:)),'b'); hold on;
      plot(tt,squeeze(sentV(9,:)),'b'); hold on;
      plot(tt,squeeze(sentV(10,:)),'b'); hold on;
      plot(tt,squeeze(sentV(11,:)),'b'); hold on;
      plot(tt,squeeze(sentV(12,:)),'b'); hold on;
       legend('Real Robot','Desired ',2);
      xlabel('Time [sec]');
      ylabel('Joint Values, radians')
      
      