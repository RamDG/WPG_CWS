% lt=0:.05:10;
% if size(dL,2)<size(lt,2)-1
%     tempd=repmat(dL(:,end),1,(size(lt,2)-size(dL,2)));
%     dL=[dL,tempd];
% end
% lt=lt(1:end-1);
% 
% Desired_ZMP_x = @(t) kajita_desired_zmp_x3d_straight(t,Tr,...
%                                                                Delta_ZMP);
%         Desired_ZMP_y = @(t) kajita_desired_zmp_y3d_straight(t,Tr,...
%                                                                Delta_ZMP);
%                                                            
%  Des_T_Cx=@(t) get_T_Cx(t,T_Cx,ddz_G,zG,y_c,z_c,lt,dL);
%  Des_T_Cy=@(t) get_T_Cy(t,T_Cx,ddz_G,zG,y_c,z_c,lt,dL);
%  
%  
%  % -----------------------------------------------------------
%  function [T_x]=get_T_Cx(t,T_Cx,ddz_G,y_c,z_c,lt,dL)
% global total_mass;
% global g;
% T_Cx = interp1(lt,T_Cx,t);
% dL_x= interp1(lt,dL(1,:),t);
% ddz_G = interp1(lt,ddz_G,t);
% y_c = interp1(lt,y_c,t);
% 
% T_x=  T_Cx- dL_x  - total_mass*(ddz_G + g).* y_c  ;
%  end
% %-------------------------------------------------------------
% function [T_y]=get_T_Cy(t,T_Cx,ddz_G,x_c,z_c,lt,dL)
% global total_mass;
% global g;
% T_Cx = interp1(lt,T_Cx,t);
% dL_y= interp1(lt,dL(2,:),t);
% ddz_G = interp1(lt,ddz_G,t);
% x_c = interp1(lt,x_c,t);
% 
% T_y= T_Cx- dL_y  + total_mass*(ddz_G + g).* x_c  ;
% end
a=1;
for ip=1:size(lambda_traj,3)
for pi=1:size(lambda_traj,2)
x(a)=sum(lambda_traj(1:4,pi,ip))*P_F(1,1,pi,ip);
x(a)=(x(a)+sum(lambda_traj(5:8,pi,ip))*P_F(1,2,pi,ip))/sum(lambda_traj(:,pi,ip));

y(a)=sum(lambda_traj(1:4,pi,ip))*P_F(2,1,pi,ip);
y(a)=(y(a)+sum(lambda_traj(5:8,pi,ip))*P_F(2,2,pi,ip))/sum(lambda_traj(:,pi,ip));


a=a+1;
end 
end

plot (x,y)
figure,
plot(x)
figure,
plot(y)
figure,
plot(T_Cx)
figure,
plot(T_Cy)