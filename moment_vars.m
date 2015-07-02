function [ x_c, y_c,z_c,T_Cx,T_Cy ] = moment_vars( ddz_G, lambda_traj,p_k, alpha, n_k )
global total_mass;
global g;
x_c=zeros(1,size(lambda_traj,2));
y_c=zeros(1,size(lambda_traj,2));
z_c=zeros(1,size(lambda_traj,2));
T_Cx=zeros(1,size(lambda_traj,2));
T_Cy=zeros(1,size(lambda_traj,2));
s=1;

for i=1:size(lambda_traj,2)
    
    % frictionless CWC variables
    eps_k = zeros(1,size(lambda_traj(:,i),1));
    
    for k = 1:size(lambda_traj(:,i),1)
        eps_k(1,k) = (1-alpha)*total_mass*(ddz_G(i) + g)*lambda_traj(k,i)...
            /sum(lambda_traj(:,i))*n_k(3,k);
    end
    
    %% sum of forces on contacts and moments
    eps=sum(eps_k(1,:));
    for k = 1:size(eps_k,2)
        x_c(s)= x_c(s) + alpha * eps_k(1,k) * p_k(1,k,i) / eps;
        y_c(s)= y_c(s) + alpha * eps_k(1,k) * p_k(2,k,i) /eps;
        z_c(s)= z_c(s) + (1-alpha) * eps_k(1,k) * p_k(3,k,i) /eps;
        T_Cx(s)= T_Cx(s) + eps_k(1,k) * ( p_k(2,k,i) * n_k(3,k) - p_k(3,k,i) * n_k(2,k) );
        T_Cy(s)= T_Cy(s) - eps_k(1,k) * ( p_k(1,k,i) * n_k(3,k) - p_k(3,k,i) * n_k(1,k) );
        
    end
    
    s=s+1;
end
