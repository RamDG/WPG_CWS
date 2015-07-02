function [dy]=diff_eqx(t,y,T_Cy,ddz_G,zG,y_c,z_c,lt,dL)
global total_mass;
global g;
T_Cy = interp1(lt,T_Cy,t);
dL_y= interp1(lt,dL(2,:),t);
ddz_G = interp1(lt,ddz_G,t);
%zG = interp1(lt,zG,t);
x_c = interp1(lt,y_c,t);
z_c = interp1(lt,z_c,t);
dy=zeros(2,1);
dy(1)=y(2);
dy(2)= ( (  -T_Cy -dL_y)/total_mass + (ddz_G + g) .* ( y(1) - x_c) ) ./(zG-z_c);
  