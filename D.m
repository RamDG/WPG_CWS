function [ Dr ] = D( r )
%D D operation as described in Kajita2003 resolved momentum control
%   r : vector to operate on

%   Dr : the resultant matrix
%% Apply the D operator
Dr = skew(r)'*skew(r);

end

