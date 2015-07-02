function [ v_constrained,ok ] = velocity_check( v, v_max )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes her
ok=true;
%v_constrained = min(v,v_max);
%TODO: 
v_constrained = min(abs(v),v_max);
v_constrained=v_constrained.*sign(v);

if sum(v_constrained-v)~=0
    ok=false;
end
end

