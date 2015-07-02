function [ws]= wrench_sum(p_C,ddp_C,dL)
global total_mass g
ws(1)=total_mass*(ddp_C(3)+g)*p_C(2)-total_mass*p_C(3)*ddp_C(2)+ dL(1);
ws(2)=-total_mass*(ddp_C(3)+g)*p_C(1)+total_mass*p_C(3)*ddp_C(1)+ dL(2);

% ws(1)=total_mass*(g)*p_C(2)-total_mass*p_C(3)*ddp_C(2)+ dL(1);
% ws(2)=-total_mass*(g)*p_C(1)+total_mass*p_C(3)*ddp_C(1)+ dL(2);