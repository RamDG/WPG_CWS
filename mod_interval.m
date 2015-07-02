function [ x_mod ] = mod_interval( x, a, b )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if a ~= b 
 x_mod = a + mod(x-a,b-a);
end

