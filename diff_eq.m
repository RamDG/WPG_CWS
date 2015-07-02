function [dy]=diff_eq(t,y,T_Cx,ddz_G,zG,y_c,z_c,lt,dL)
global total_mass;
global g;
T_Cx = interp1(lt,T_Cx,t);
dL_x= interp1(lt,dL(1,:),t);
ddz_G = interp1(lt,ddz_G,t);
%zG = interp1(lt,zG,t);
y_c = interp1(lt,y_c,t);
z_c = interp1(lt,z_c,t);
dy=zeros(2,1);
dy(1)=y(2);

dy(2)=( ( dL_x + T_Cx )/total_mass + (ddz_G + g) .* ( y(1) - y_c) ) ./(zG-z_c);
