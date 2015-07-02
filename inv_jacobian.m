function [ inv_J ] = inv_jacobian( J )
%INV_JACOBIAN inverse the Jacobian matrix J using SVD decomposition
%   J : Jacobian matrix to inverse

%% Calculate the SVD of J
[U,S,V] = svd(J);
i=min(size(S));
iS = zeros (size (S));
for t=1:i
    if ( S(t,t)< .00001 )
        S(t,t)=S(t,t)+.00001;
        iS(t,t)=S(t,t)/(S(t,t)^2+.000001);
        display('reaching singularity');
    else
        iS(t,t)=S(t,t)/(S(t,t)^2);
    end
end

%% Calculate the inverse
% inv_J = V/S*U';
inv_J = V*iS'*U';
end

