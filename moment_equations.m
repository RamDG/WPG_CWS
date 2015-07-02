function [ x_Gnext, y_Gnext ] = moment_equations( zG, p_G, p_Gprev, ddz_G, lambda_ref, dL, p_k, alpha, n_k )
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

x_Gprev= p_Gprev(1);
x_G= p_G(1);
y_Gprev= p_Gprev(2);
y_G= p_G(2);

%% Force magnitude calculation
eps_k = zeros(1,size(lambda_ref,2));

for k = 1:size(lambda_ref,2)
    eps_k(1,k) = (1-alpha)*total_mass*(ddz_G + g)*lambda_ref(1,k)...
        /sum(lambda_ref(1,:))*n_k(3,k);
end
x_c=0;
y_c=0;
z_c=0;
T_Cx=0;
T_Cy=0;
%% sum of forces on contacts and moments
eps=sum(eps_k(1,:));
for k = 1:size(lambda_ref,2)
    x_c= x_c + alpha * eps_k(1,k) * p_k(1,k) / eps;
    y_c= y_c + alpha * eps_k(1,k) * p_k(2,k) /eps;
    z_c= z_c + (1-alpha) * eps_k(1,k) * p_k(3,k) /eps;
    T_Cx= T_Cx + eps_k(1,k) * ( p_k(2,k) * n_k(3,k) - p_k(3,k) * n_k(2,k) );
    T_Cy= T_Cy - eps_k(1,k) * ( p_k(1,k) * n_k(3,k) - p_k(3,k) * n_k(1,k) );
    
end

    %% Calculation of center of gravity in the next point using a difference equation (Nishiwaki et. al 2002)
    %this finds -Mddx since we use the -fG and -tG elements
%     y_Gnext= ( ( dL(1) - T_Cx )/total_mass + (ddz_G + g) * ( y_G - y_c) ) * dt^2 /(zG-z_c) + 2*y_G - y_Gprev;
%     
%     x_Gnext= ( ( T_Cy - dL(2) )/total_mass + (ddz_G + g) * ( x_G -x_c ) ) * dt^2 /(zG-z_c) + 2*x_G - x_Gprev;

%     % the negative of the previous    
   y_Gnext= -1*( ( ( dL(1) - T_Cx )/total_mass - (ddz_G + g) * ( y_G - y_c) ) * dt^2 /(zG-z_c) - 2*y_G + y_Gprev);
    
   x_Gnext= -1*( ( ( T_Cy - dL(2) )/total_mass + (ddz_G + g) * ( x_G -x_c ) ) * dt^2 /(zG-z_c) - 2*x_G + x_Gprev);

    ddx=(x_Gnext -2*x_G +x_Gprev)/dt^2;
    ddy=(y_Gnext -2*y_G + y_Gprev)/dt^2;
end

