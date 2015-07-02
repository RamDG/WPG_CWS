function [ r ] = skew( b )
%SKEW return the skew simmetric matrix of the vector
%   b : vector in input

%   r : skew simmetric matrix of the vector

%% Calculate the skew simmetric matrix
r = [0, -b(3), b(2); b(3) 0 -b(1); -b(2), b(1), 0];  

end