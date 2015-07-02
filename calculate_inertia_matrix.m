function [ M, H, I_hat,m_hat_1,c_hat_1 ] = calculate_inertia_matrix( m_tot, c_tot, I_tot, R_tot, r )
%INERTIA_MATRIX calculate the inertia matrix of the robot
%   m_tot : masse of each joints of the body
%   c_tot : com of each joints of the body referred to their local frame
%   I_tot : inertia matrices of each joints of the body in their local
%   frame
%   R_tot : rotation matrices from each joints to the base (torso frame)
%   r     : position of the joint with respect to the base frame

%%  Calculation of the total masses and inertia matrices of the entire body
% initialisation
m_hat = zeros(1,length(m_tot));
c_hat = zeros(3,length(m_tot));
I_hat = zeros(3,3,length(m_tot));

% initialisation with the end effector
m_hat(end) = m_tot(end);
c_hat(:,end) = R_tot(:,:,end)*c_tot(:,end)+r(:,end); %0R_end*c+r to get com with respect to base frame
I_hat(:,:,end) = I_tot(:,:,end);

% loop through all the joints of the body using R matrix wrt to previous
% joint
% for j = (length(m_tot)-1):-1:2
%     R=R_tot(:,:,j-1)'*R_tot(:,:,j);
%     m_hat(j) = m_hat(j+1) + m_tot(j);
%     c_hat(:,j) = (m_hat(j+1)*c_hat(:,j+1) + m_tot(j)*(R_tot(:,:,j)*c_tot(:,j)+r(:,j))) / ...
%         m_hat(j);
%     I_hat(:,:,j) = I_hat(:,:,j+1) + m_hat(j+1)*D(c_hat(:,j+1)-c_hat(:,j)) + ...
%         R*I_tot(:,:,j)*R' + m_tot(j)*D((R_tot(:,:,j)*c_tot(:,j)+r(:,j))-c_hat(:,j));
% end

% loop through all the joints of the body using R matrix wrt base frame
for j = (length(m_tot)-1):-1:1
    m_hat(j) = m_hat(j+1) + m_tot(j);
    c_hat(:,j) =( m_hat(j+1)*c_hat(:,j+1) + m_tot(j)*( R_tot(:,:,j)*c_tot(:,j) + r(:,j) )) / ...
        m_hat(j);
    I_hat(:,:,j) = I_hat(:,:,j+1) + m_hat(j+1)*D(c_hat(:,j+1)-c_hat(:,j)) + ...
        R_tot(:,:,j)*I_tot(:,:,j)*R_tot(:,:,j)' + m_tot(j)*D((R_tot(:,:,j)*c_tot(:,j)+r(:,j))-c_hat(:,j));
end


%% Calculation of the matrices M and H
M = zeros(3,length(m_hat));
H0 = zeros(3,length(m_hat));

% loop through all the joints and calculate M
for j = 1:length(m_hat)
    M(:,j) = cross(R_tot(1:3,3,j),((c_hat(:,j)-r(:,j))*m_hat(j)));
    H0(:,j) = cross(c_hat(:,j),M(:,j)) + I_hat(:,:,j)*R_tot(1:3,3,j);
end
% calculate H using M and H0
H = H0;
% H = H0 - skew(c_hat(:,1))*M; %to the com of the torso
%TODO: check which of the two is more convenient
% global P_B_COG % TODO: ensure P_B_COG has the current value
% H = H0 - skew(P_B_COG)*M; %to the COG of the robot
m_hat_1=m_hat(1);
c_hat_1=c_hat(:,1);