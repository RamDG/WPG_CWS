function [CWS, CoG, CoGD] = PreviewControl(Des_T_Cx,...
    Des_T_Cy,TSimulation,zG,z_c,X0,X0Dot,Y0,Y0Dot)
global step_time step_length dt
global total_mass g
plt=1; % 1 to plot ws results

% Parameters
M=total_mass;
% h = Tr/Delta_ZMP/2/1000; % discretization time
h = dt/1;
% Discrete state space model
Plantx = stateMomentx(total_mass,g,zG,z_c,h);
Planty = stateMomenty(total_mass,g,zG,z_c,h);

% Parameters of the control law
Q = 1;
R = 1E-6;
N = step_time*2/(h);   % Look into 2 steps into the future
[Kx,Fx]= compute_preview_control(Plantx,Q,R,N);
[Ky,Fy]= compute_preview_control(Planty,Q,R,N);
% Reference
[Mr,Br,NXRef0,NYRef0] = get_next_n_inputs(Des_T_Cx,Des_T_Cy,...
    N,Plantx.Ts);

% Initial conditions
h = Plantx.Ts;

Simulink_File = 'sim_kajita_closed_loop';
%Simulink_File = 'mo_eq_closed_loop'; % to use this the gains k have to be
%changed to something similar to paper zmp with preview control kajita 2003

% Note: need to use another...
%simulink file that has what its required for CWS
Options = simget(Simulink_File);
Options = simset(Options,'DstWorkspace','current','SrcWorkspace',...
    'current');
sim(Simulink_File,TSimulation,Options);
if plt==1
    figure,
    plot(CWSref.time(:,1),CWSref.signals.values(:,1), 'r', 'Linewidth', 2);
    hold on
    plot (CWS.time(:,1),CWS.signals.values(:,1), 'g' ,'Linewidth', 2);
    % hold on
    % plot (CoG.time(:,1),CoG.signals.values(:,1),  'Linewidth', 2);
    %
    % legend('CWSref','CWS','CoG',2);
    legend('T_Cx_ref','T_Cx',2);
    xlabel('Time [sec]');
    ylabel('T_Cy, N*meters')
    grid on;
    
    figure,
    plot(CWSref.time,CWSref.signals.values(:,2), 'r','Linewidth', 2);
    hold on
    plot (CWS.time,CWS.signals.values(:,2), 'g','Linewidth', 2);
    % hold on
    % plot (CoG.time,CoG.signals.values(:,2),'Linewidth', 2);
    
    % legend('CWSref','CWS','CoG',2);
    legend('T_Cy_ref','T_Cy',2);
    xlabel('Time [sec]');
    ylabel('T_Cx, N*meters')
    grid on;
    
end
end

%---------------Additional_Functions-----------------------------------

%----------------State equation y--------------------------------------
function [Planty] = stateMomenty(M,g,zG,z_c,h)
% Continuous time model   
A = [0, 1, 0;
    0, 0, 1; 
    0, 0, 0]; %  tracking control
B = [0; 0; 1];
C = [M*g, 0, -M*(zG-z_c)]; 
D = 0;
Planty = c2d( ss(A,B,C,D) , h, 'zoh');
end
% 

%----------------State equation x--------------------------------------
function [Plantx] = stateMomentx(M,g,zG,z_c,h)
% Continuous time model   
A = [0, 1, 0;
    0, 0, 1;
    0, 0, 0]; %  tracking control
B = [0; 0; 1];
C = [-M*g, 0, M*(zG-z_c)]; 
D = 0;
Plantx = c2d( ss(A,B,C,D) , h, 'zoh');
end
% 
%-----------------Compute_preview_control------------------------------

function [K,F]= compute_preview_control(Plant, Q, R, N)
% [K,F] = COMPUTE_PREVIEW_CONTROL(Plant, Q, R, N) calculates the
% control law
%                                           [yref(k + 1)]
%             u(k) = -K*x(k) + [f1 .. fN] * [    ...    ]
%                                           [yref(k + N)]
% minimizing
%
%                J = Sum {(yref-y)'Q(yref-y) + du'Rdu}
%
% Plant is supposed given by
%
%           x(k+1) = Ax(k) + Bu(k)
%             y(k) = Cx(k) + Du(k)
%
% For the optimization yref is considered as a constant function
A = Plant.A;
B = Plant.B;
C = Plant.C;
D = Plant.D;
h = Plant.Ts;

% Computing K
[K,P]= lqry(Plant,Q,R,0);

% P and K should verify the following equalities
%     Test_P      = P - (A'*P*A + C'*Q*C - A'*P*B * inv(R + B'*P*B) ...
                                                            %* B' * P * A)
%     Test_K      = K - (inv(R + B'*P*B) * B' * P * A)

F = zeros(N,1);
for i = 1:N,
    F(i) = (R + B'*P*B)\(B' * mpower((A - B*K)',i-1) * C' * Q);
end

% Closed loop DC gain
Gain_SS = dcgain(ss(A-B*K,B,C,D,h)) * sum(F);
F = F/Gain_SS;
end
% 
% -------------get_next_n_inputs----------------------------------------------
function [Mr,Br,NXRef0,NYRef0] = get_next_n_inputs(get_reference_x,...
                                                    get_reference_y, N, h)
% Parameters for the reference 
%
% NYRef(k) = Mr * NYRef(k-1) + B * yref(k+N)
%
% with NYRef(k) = [ yref(k+1); .... yref(k+N)];
%
Mr = [zeros(N-1,1), eye(N-1); zeros(1,N-1), 0;];
Br = [zeros(N-1,1); 1];

NXRef0 = get_reference_x((0:N-1)'*h );
NYRef0 = get_reference_y((0:N-1)'*h );

end
