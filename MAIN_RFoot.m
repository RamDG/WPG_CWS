%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                                                                         %
%                       NAO robot kinematic model                         %
%                              with masses                                %
%                                                                         %
%                              Louise Poubel                              %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% NAO H25

clear
clc

% PARAMETERS

SaveFig = false;

% 'plot'
% 'symbolic'
symbolic = 'symbolic';

% 'justKinematics'
% 'masses'
% 'justOrigins'
% 'bodyParts'
KinematicChain = 'bodyParts';

% The way of choosing above is not clever. 
% More flexibility should be given to the user by allowing to turn on or off each of things being plotted.


dpi = '-r500';
linewidth = 1.2;






set(0,'defaultfigureposition',[100 100 500 500]))

if strcmp(KinematicChain,'bodyParts')
    load('NAO/HeadPitch.mat');
    load('NAO/Torso.mat');
    load('NAO/LHipPitch.mat');
    load('NAO/RHipPitch.mat');
    load('NAO/LKneePitch.mat');
    load('NAO/RKneePitch.mat');
    load('NAO/LAnkleRoll.mat');
    load('NAO/RAnkleRoll.mat');
    load('NAO/LShoulderRoll.mat');
    load('NAO/RShoulderRoll.mat');
    load('NAO/LElbowRollComplete.mat');
    load('NAO/RElbowRollComplete.mat');
end

%%


% For symbolic development
if strcmp(symbolic,'symbolic')
    q1=sym('q1'); q2=sym('q2'); q3=sym('q3'); q4=sym('q4'); q5=sym('q5'); q6=sym('q6');
    q7=sym('q7'); q8=sym('q8'); q9=sym('q9'); q10=sym('q10'); q11=sym('q11');
    q12=sym('q12'); q13=sym('q13'); q14=sym('q14'); q15=sym('q15');
    q16=sym('q16'); q17=sym('q17'); q18=sym('q18'); q19=sym('q19');
    q20=sym('q20'); q21=sym('q21'); q22=sym('q22'); q23=sym('q23');
    q=[q1;q2;q3;q4;q5;q6;q7;q8;q9;q10;q11;q12;q13;q14;q15;q16;q17;q18;q19;q20;q21;q22;q23];
    q = sym(q);
end

% For numeric development
% To see a specific configuration, assign the values to q.
if strcmp(symbolic,'plot')
    q = zeros(1,23);
    
%     q(1) = 7.4*pi/180;
%     q(2) = 5.2*pi/180;
%     q(3) = -5.2*pi/180;
%     q(4) = 7.4*pi/180;
%     q(5) = -5.7*pi/180;
%     q(6) = -9.7*pi/180;
%     q(7) = 5.7*pi/180;
%     q(8) = 7.4*pi/180;
%     q(9) = -5.2*pi/180;
%     q(10) = 5.2*pi/180;
%     q(11) = -7.4*pi/180;
% 
%      q(3) = pi/2;
%     q(9) = pi/2;
%     q(13) = -pi/6;
%     q(15) = pi/4;
%     q(6) = pi/3;

%     Get_q_from_Results_Obtained;
%     q(12:23) = qALL(1,12:23);
%     q(17) = q(12);

end


% File to load the parameters
PARAMETERS_RFoot;



%%
linewidth = 1;
% Plot if not symbolic
if strcmp(symbolic,'plot')
    FIG = figure(168);
    clf
    hold on
    xlabel('x','FontSize',14);
    ylabel('y','FontSize',14);
    zlabel('z','FontSize',14);
    set(gca,'FontSize',14);
    light('Position',[1 1 1])
    axis vis3d equal;
    view(135,19);
    %view(127,5);
    set(gcf,'color','w');
    xlim([-200,200]);
    ylim([-200,200]);
    zlim([-100,600]);
    axis off
    
    Transparency = 1;
    if strcmp(KinematicChain,'masses')
        Transparency = 0.2;
    end
    
    % Frame B(0)
    % Find the successors to plot their U axis
    gammas = [0];
    bs = [0];
    k = 2;
    for l = 1:length(a)
        if a(l) == 0
            gammas(k) = gamma(l);
            bs(k) = b(l);
            k = k + 1;
        end
    end
%     Fig_PlotAxis([0,0,0],[0,0,0],false,Transparency,[gammas',bs'],KinematicChain);
    for j = 1:length(a)
        gammas = [0];
        bs = [0];
        k = 2;
        % Find the successors to plot their U axis
        for l = 1:length(a)
            if a(l) == j
                gammas(k) = gamma(l);
                bs(k) = b(l);
                k = k + 1;
            end
        end
        % Fixed frames have thin x-y-z
        if sigma(j) == 2
            Fig_PlotAxis(T(:,:,j),[],false,Transparency,[gammas',bs'],KinematicChain);
        end
        if sigma(j) == 0
            % DOFs have a thick z and a thin x
            Fig_PlotAxis(T(:,:,j),[],true,Transparency,[gammas',bs'],KinematicChain);
        end
        % Skeleton (line from j to antecedent)
        if a(j) ~= 0
            frameA = j; frameB = a(j);
            line([T(1,4,frameA) T(1,4,frameB)],[T(2,4,frameA) T(2,4,frameB)],[T(3,4,frameA) T(3,4,frameB)],'Color','k','LineStyle','--','LineWidth',linewidth);
        else
            frameA = j;
            line([T(1,4,frameA) 0],[T(2,4,frameA) 0],[T(3,4,frameA) 0],'Color','k','LineStyle','--','LineWidth',linewidth);
        end
    end
    % Tracked point on LFoot tip
    [X Y Z]=sphere(20); % Also used for masses
    k = 14;
    T_LFoot = T(:,:,k);
    P_track = T_LFoot*[100;0;0;1];
    Sphere = surf(P_track(1)+X*5,P_track(2)+Y*5,P_track(3)+Z*5);
    set(Sphere,'FaceLighting','phong','EdgeLighting','phong');
    set(Sphere,'EraseMode','normal','EdgeColor','none','FaceColor','g','FaceAlpha',Transparency);
        
    
    % Skeleton of torso
    frameA = 6; frameB = 9;
    line([T(1,4,frameA) T(1,4,frameB)],[T(2,4,frameA) T(2,4,frameB)],[T(3,4,frameA) T(3,4,frameB)],'Color','k','LineStyle','--','LineWidth',linewidth);
    frameA = 9; frameB = 22;
    line([T(1,4,frameA) T(1,4,frameB)],[T(2,4,frameA) T(2,4,frameB)],[T(3,4,frameA) T(3,4,frameB)],'Color','k','LineStyle','--','LineWidth',linewidth);
    frameA = 22; frameB = 16;
    line([T(1,4,frameA) T(1,4,frameB)],[T(2,4,frameA) T(2,4,frameB)],[T(3,4,frameA) T(3,4,frameB)],'Color','k','LineStyle','--','LineWidth',linewidth);
    frameA = 16; frameB = 6;
    line([T(1,4,frameA) T(1,4,frameB)],[T(2,4,frameA) T(2,4,frameB)],[T(3,4,frameA) T(3,4,frameB)],'Color','k','LineStyle','--','LineWidth',linewidth);
    frameA = 7; frameB = 6; frameC = 9;
    line([T(1,4,frameA) (T(1,4,frameB)+T(1,4,frameC))/2],[T(2,4,frameA) (T(2,4,frameB)+T(2,4,frameC))/2],[T(3,4,frameA) (T(3,4,frameB)+T(3,4,frameC))/2],'Color','k','LineStyle','--','LineWidth',linewidth);
    
    if strcmp(KinematicChain,'masses')
        Transparency = 1;
        MassScale = 60;
        Sphere = surf(c_0_0(1)+X*m0*MassScale,c_0_0(2)+Y*m0*MassScale,c_0_0(3)+Z*m0*MassScale);
        set(Sphere,'FaceLighting','phong','EdgeLighting','phong');
        set(Sphere,'EraseMode','normal','EdgeColor','none','FaceColor','y','FaceAlpha',Transparency);
        for j = 1:length(m)
            if m(j)~=0
                Sphere = surf(c_0(j,1)+X*m(j)*MassScale,c_0(j,2)+Y*m(j)*MassScale,c_0(j,3)+Z*m(j)*MassScale);
                set(Sphere,'FaceLighting','phong','EdgeLighting','phong');
                set(Sphere,'EraseMode','normal','EdgeColor','none','FaceColor','y','FaceAlpha',Transparency);
            end
        end
        plot3(CoM(1),CoM(2),CoM(3),'r+','LineWidth',5,'MarkerSize',20);
        plot3(CoM(1),CoM(2),0,'k+','LineWidth',5,'MarkerSize',20);
    end
    
    if strcmp(KinematicChain,'bodyParts')
        Transparency = 0.3;
        MassScale = 60;
        % RAnkleRoll
        for i = 1: length(VRAnkleRoll(:,1))
            NewVRAnkleRoll(:,i) = BuildT(0,0,0,0,0,0)*[VRAnkleRoll(i,:) 1]';
        end
        p1 = patch('faces', FRAnkleRoll, 'vertices' ,NewVRAnkleRoll(1:3,:)');
        set(p1, 'facec', 'w','FaceAlpha',Transparency);
        set(p1, 'FaceVertexCData', [1 1 1 ]);
        set(p1, 'EdgeColor','none');
        
        for j = 1:length(m)
            % HeadPitch
            if j == 28
                for i = 1: length(VHeadPitch(:,1))
                    NewVHeadPitch(:,i) = T(:,:,j)*BuildT(0,0,0,-90,0,-90)*[VHeadPitch(i,:) 1]';
                end
                p1 = patch('faces', FHeadPitch, 'vertices' ,NewVHeadPitch(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                set(p1, 'EdgeColor','none');
            end
            % Torso
            if j == 7
                for i = 1: length(VTorso(:,1))
                    NewVTorso(:,i) = T(:,:,j)*BuildT(0,0,0,0,0,0)*[VTorso(i,:) 1]';
                end
                p1 = patch('faces', FTorso, 'vertices' ,NewVTorso(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                set(p1, 'EdgeColor','none');
            end
            % LHipPitch
            if j == 10
                for i = 1: length(VLHipPitch(:,1))
                    NewVLHipPitch(:,i) = T(:,:,j)*BuildT(0,0,0,90,180,90)*[VLHipPitch(i,:) 1]';
                end
                p1 = patch('faces', FLHipPitch, 'vertices' ,NewVLHipPitch(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % RHipPitch
            if j == 3
                for i = 1: length(VRHipPitch(:,1))
                    NewVRHipPitch(:,i) = T(:,:,j)*BuildT(ThighLength,0,0,-90,0,-90)*[VRHipPitch(i,:) 1]';
                end
                p1 = patch('faces', FRHipPitch, 'vertices' ,NewVRHipPitch(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % LKneePitch
            if j == 11
                for i = 1: length(VLKneePitch(:,1))
                    NewVLKneePitch(:,i) = T(:,:,j)*BuildT(0,0,0,90,180,90)*[VLKneePitch(i,:) 1]';
                end
                p1 = patch('faces', FLKneePitch, 'vertices' ,NewVLKneePitch(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % RKneePitch
            if j == 2
                for i = 1: length(VRKneePitch(:,1))
                    NewVRKneePitch(:,i) = T(:,:,j)*BuildT(TibiaLength,0,0,-90,0,-90)*[VRKneePitch(i,:) 1]';
                end
                p1 = patch('faces', FRKneePitch, 'vertices' ,NewVRKneePitch(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % LAnkleRoll
            if j == 13
                for i = 1: length(VLAnkleRoll(:,1))
                    NewVLAnkleRoll(:,i) = T(:,:,j)*BuildT(0,0,0,0,90,-90)*[VLAnkleRoll(i,:) 1]';
                end
                p1 = patch('faces', FLAnkleRoll, 'vertices' ,NewVLAnkleRoll(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % LShoulderRoll
            if j == 22
                for i = 1: length(VLShoulderRoll(:,1))
                    NewVLShoulderRoll(:,i) = T(:,:,j)*BuildT(0,0,0,0,0,90)*[VLShoulderRoll(i,:) 1]';
                end
                p1 = patch('faces', FLShoulderRoll, 'vertices' ,NewVLShoulderRoll(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % RShoulderRoll
            if j == 16
                for i = 1: length(VRShoulderRoll(:,1))
                    NewVRShoulderRoll(:,i) = T(:,:,j)*BuildT(0,0,0,0,0,90)*[VRShoulderRoll(i,:) 1]';
                end
                p1 = patch('faces', FRShoulderRoll, 'vertices' ,NewVRShoulderRoll(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % LElbowRoll
            if j == 24
                for i = 1: length(VLElbowRollComplete(:,1))
                    NewVLElbowRollComplete(:,i) = T(:,:,j)*BuildT(0,0,0,0,0,90)*[VLElbowRollComplete(i,:) 1]';
                end
                p1 = patch('faces', FLElbowRollComplete, 'vertices' ,NewVLElbowRollComplete(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
            % RElbowRoll
            if j == 18
                for i = 1: length(VRElbowRollComplete(:,1))
                    NewVRElbowRollComplete(:,i) = T(:,:,j)*BuildT(0,0,0,0,0,90)*[VRElbowRollComplete(i,:) 1]';
                end
                p1 = patch('faces', FRElbowRollComplete, 'vertices' ,NewVRElbowRollComplete(1:3,:)');
                set(p1, 'facec', 'w','FaceAlpha',Transparency);
                
                set(p1, 'EdgeColor','none');
                
            end
        end
        plot3(CoM(1),CoM(2),CoM(3),'r+','LineWidth',5,'MarkerSize',20);
        % Back to torso reference for the plot
        plot3(CoM(1),CoM(2),0,'k+','LineWidth',5,'MarkerSize',20);
    end
    
    if SaveFig
        %print(FIG,'-dpng',dpi,['figs/',KinematicChain,'_RFoot'])
        print(FIG,'-dpng',dpi,['figs/','DOF','_RFoot'])
    end
end



if strcmp(symbolic,'symbolic')
    % DIRECT KINEMATICS -- SYMBOLIC
    
    % Left Ankle
    k = 12;
    P_LAnkle = T(1:3,4,k);
    P_LAnkle = vpa(P_LAnkle,4); % Represent as decimals instead of fractions
    
    % Left Foot sole
    k = 14;
    P_LFoot = T(1:3,4,k);
    P_LFoot = vpa(P_LFoot,4); % Represent as decimals instead of fractions
    
    % Right Hand
    k = 20;
    P_RHand = T(1:3,4,k);
    P_RHand = vpa(P_RHand,4); % Represent as decimals instead of fractions
    
    % Left Hand
    k = 26;
    P_LHand = T(1:3,4,k);
    P_LHand = vpa(P_LHand,4); % Represent as decimals instead of fractions
    
    
%     
%     
%     
%     clc
%     diary txt_RFoot/P_LAnkle.txt
%     ccode(P_LAnkle) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     clc
%     diary txt_RFoot/P_LFoot.txt
%     ccode(P_LFoot) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     clc
%     diary txt_RFoot/P_RHand.txt
%     ccode(P_RHand) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     clc
%     diary txt_RFoot/P_LHand.txt
%     ccode(P_LHand) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     
%     %CoM[x,y,z]
%     P_CoM = transpose(CoM);
%     P_CoM = vpa(P_CoM,4);
%     
%     clc
%     diary txt_RFoot/P_CoM.txt
%     ccode(P_CoM) % C friendly equations (still have to change the brackets for indexing)
%     diary off
% 

%     % The diagonal of the LFoot rotation matrix has to be = 1
%     % Left Foot sole
%     k = 14;
%     R_LFoot = [T(1,1,k);T(2,2,k);T(3,3,k)];
%     R_LFoot = vpa(R_LFoot,4); % Represent as decimals instead of fractions
%     
%     clc
%     diary txt_RFoot/R_LFoot.txt
%     ccode(R_LFoot) % C friendly equations (still have to change the brackets for indexing)
%     diary off
    
    % Track a point below of the ankle
    % Left Foot sole
    k = 14;
    T_LFoot = T(:,:,k);
    P_track = T_LFoot*[0;0;0;1];
    P_track = P_track(1:3);
    
    clc
    diary txt_RFoot/P_track.txt
    ccode(P_track) % C friendly equations (still have to change the brackets for indexing)
    diary off

    JacobianP_track =jacobian(P_track,q);
    JacobianP_track = vpa(JacobianP_track,4);
    
    clc
    diary txt_RFoot/JacobianP_track.txt
    ccode(JacobianP_track) % C friendly equations (still have to change the brackets for indexing)
    diary off

    
%     % JACOBIANS
%     
%     % Left Ankle
%     JacobianLAnkle=jacobian(P_LAnkle,q);
%     JacobianLAnkle = vpa(JacobianLAnkle,4);
%     
%     clc
%     diary txt_RFoot/JacobianLAnkle.txt
%     ccode(JacobianLAnkle) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     
%     % Right Hand
%     JacobianRHand=jacobian(P_RHand,q);
%     JacobianRHand = vpa(JacobianRHand,4);
%     
%     clc
%     diary txt_RFoot/JacobianRHand.txt
%     ccode(JacobianRHand) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     
%     % Left Hand
%     JacobianLHand=jacobian(P_LHand,q);
%     JacobianLHand = vpa(JacobianLHand,4);
%     
%     clc
%     diary txt_RFoot/JacobianLHand.txt
%     ccode(JacobianLHand) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     % CoMxy
%     JacobianCoMxy=jacobian(P_CoM(1:2),q);
%     JacobianCoMxy = vpa(JacobianCoMxy,4);
%     
%     clc
%     diary txt_RFoot/JacobianCoMxy.txt
%     ccode(JacobianCoMxy) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
%     % Left Foot
%     JacobianZ_LFoot=jacobian(P_LFoot(3),q);
%     JacobianZ_LFoot = vpa(JacobianZ_LFoot,4);
%     
%     clc
%     diary txt_RFoot/JacobianZ_LFoot.txt
%     ccode(JacobianZ_LFoot) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
% 
%     % Left Foot
%     JacobianR_LFoot =jacobian(R_LFoot,q);
%     JacobianR_LFoot = vpa(JacobianR_LFoot,4);
%     
%     clc
%     diary txt_RFoot/JacobianR_LFoot.txt
%     ccode(JacobianR_LFoot) % C friendly equations (still have to change the brackets for indexing)
%     diary off
%     
end
