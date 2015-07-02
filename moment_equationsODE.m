function [ x, y ] = moment_equationsODE( zG, ddz_G,  dL, x_c, y_c,z_c,T_Cx,T_Cy,P_COG ) %add initial velocity?
%MOMENT EQUATIONS solve the differential equations to satisfy the
%stability using CWS criterion
%   ddz_g : acceleration of the COG in z-axis
%   lambda : vector of scalars with at least 3 values between 0 and 1, used
%   to get the magnitude of forces
%   dL : differnetiate of the angular momentum
%   p_k : position of contact points measured from reference 
%   alpha : coefficient to disrtibute normal forces to friction forces
%   n_k : unit normal vector at contacts

%   ddx_g : acceleration of the COG in x-axis
%   ddy_g : acceleration of the COG in y-axis
%TODO: change to be able to use any alpha and contact with uneven floor
%% Global variables
global g;
global total_mass;
global dt;
%lt=0:dt:10;
% lt=0:.05:10;
% if size(dL,2)<size(lt,2) %Can use this loop to add 1.6s of not moving for
% the PC control
%     tempd=repmat(dL(:,end),1,(size(lt,2)-size(dL,2)));
%     dL=[dL,tempd];
% end

lt=0:dt:10;

lt=lt(1:end-1);

[Des_T_Cx]=@(t)get_T_Cx(t,T_Cx,ddz_G,y_c,lt,dL);
[Des_T_Cy]=@(t)get_T_Cy(t,T_Cy,ddz_G,x_c,lt,dL);
TSimulation=8;
X0=P_COG(1);
Y0=P_COG(2);
% [TC, CoG] = PreviewControl(Des_T_Cx,...
  %                        Des_T_Cy,TSimulation,zG,z_c,X0,X0Dot,Y0,Y0Dot);
  [TC, CoG, CoGD] = PreviewControl(Des_T_Cx,...
                          Des_T_Cy,TSimulation,zG,0,X0,0,Y0,0);
temp=1:10.0563:1609;
temp=fix(temp);
y(:,1)=CoG.signals.values(temp,1);
x(:,1)=CoG.signals.values(temp,2);
y(:,2)=CoGD.signals.values(temp,1);
x(:,2)=CoGD.signals.values(temp,2);
end

 % -----------------------------------------------------------
 function [T_x]=get_T_Cx(t,T_Cx,ddz_G,y_c,lt,dL)
global total_mass;
global g;
T_Cx = interp1(lt,T_Cx,t);
dL_x= interp1(lt,dL(1,:),t);
ddz_G = interp1(lt,ddz_G,t);
y_c = interp1(lt,y_c,t);

T_x=  T_Cx- dL_x  + total_mass*(ddz_G + g).* y_c  ;
 end
%-------------------------------------------------------------
function [T_y]=get_T_Cy(t,T_Cy,ddz_G,x_c,lt,dL)
global total_mass;
global g;
T_Cy = interp1(lt,T_Cy,t);
dL_y= interp1(lt,dL(2,:),t);
ddz_G = interp1(lt,ddz_G,t);
x_c = interp1(lt,x_c,t);

T_y= T_Cy- dL_y  - total_mass*(ddz_G + g).* x_c  ;
end