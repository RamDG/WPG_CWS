function [ TC, x, y ] = solveMomentEq2( zG, ddz_G,  dL, x_c, y_c,z_c,T_Cx,T_Cy,P_COG ) %add initial velocity?
%MOMENT EQUATIONS solve the differential equations to satisfy the
%stability using CWS criterion
% Inputs:
%   ddz_g : acceleration of the COG in z-axis
%   lambda : vector of scalars with at least 3 values between 0 and 1, used
%   to get the magnitude of forces
%   dL : differnetiate of the angular momentum
%   p_k : position of contact points measured from reference 
%   alpha : coefficient to disrtibute normal forces to friction forces
%   n_k : unit normal vector at contacts

% Outputs
%   TC : Contact torque + derivative of angular momentum
%   CoG : Position of the center of gravity for all trajectory
%   CoG : Velocity of the center of gravity for all trajectory
%TODO: change to be able to use any alpha and contact with uneven floor
%TODO: Consider adding 1.6s of the robot in stationary position for the end
%of the sequence. (1.6 is the time in the future that PC can see)
%% Global variables
global dt;
global Ts


lt=0:dt:Ts;

[Des_T_Cx]=@(t)get_T_Cx(t,T_Cx,ddz_G,y_c,lt,dL);
[Des_T_Cy]=@(t)get_T_Cy(t,T_Cy,ddz_G,x_c,lt,dL);
%TSimulation=8; %this will change it should be stepTime*numberOfSteps
X0=P_COG(1);
Y0=P_COG(2);

% [TC, CoG] = PreviewControl(Des_T_Cx,...
  %                        Des_T_Cy,TSimulation,zG,z_c,X0,X0Dot,Y0,Y0Dot);
  [TC, CoG, CoGD] = PreviewControl(Des_T_Cx,...
                          Des_T_Cy,(Ts+dt),zG,0,X0,0,Y0,0);
                      
%    tt=fix(size(CoG.signals.values,1)/size(lt,2));                   
% temp=1:tt:size(CoG.signals.values,1);
% temp=fix(temp);
% y(:,1)=CoG.signals.values(temp,1);
% x(:,1)=CoG.signals.values(temp,2);
% y(:,2)=CoGD.signals.values(temp,1);
% x(:,2)=CoGD.signals.values(temp,2);

 y(:,1)  = interp1(CoG.time',CoG.signals.values(:,1),lt);
  y(:,2)  = interp1(CoGD.time',CoGD.signals.values(:,1),lt);  
  
x(:,1)  = interp1(CoG.time',CoG.signals.values(:,2),lt);  
x(:,2)  = interp1(CoGD.time',CoGD.signals.values(:,2),lt);  
end

 % -----------------------------------------------------------
 function [T_x]=get_T_Cx(t,T_Cx,ddz_G,y_c,lt,dL)
global total_mass;
global g;
global Ts
if t<=Ts
T_Cx = interp1(lt,T_Cx,t);
dL_x= interp1(lt,dL(1,:),t);
ddz_G = interp1(lt,ddz_G,t);
y_c = interp1(lt,y_c,t);
else
    T_Cx = ones(size(t))*interp1(lt,T_Cx,Ts);
dL_x= zeros(size(t));
ddz_G = zeros(size(t));
y_c = ones(size(t))*interp1(lt,y_c,Ts);
end
T_x=  T_Cx- dL_x  + total_mass*(ddz_G + g).* y_c  ;
 end
%-------------------------------------------------------------
function [T_y]=get_T_Cy(t,T_Cy,ddz_G,x_c,lt,dL)
global total_mass;
global g;
global Ts
if t<=Ts
T_Cy = interp1(lt,T_Cy,t);
dL_y= interp1(lt,dL(2,:),t);
ddz_G = interp1(lt,ddz_G,t);
x_c = interp1(lt,x_c,t);
else
    T_Cy = ones(size(t))*interp1(lt,T_Cy,Ts);
    dL_y= zeros(size(t));
    ddz_G = zeros(size(t));
    x_c = ones(size(t))*interp1(lt,x_c,Ts);
end

T_y= T_Cy- dL_y  - total_mass*(ddz_G + g).* x_c  ;
end