function [qd]=avoidJointLimits(q,e,J,invJ,limits,q1)
%Based on "New Strategies for avoiding robot joint limits: ...
%Application to visual servoing using a lare projection operator" by
%Mohammed Mare and Francois Chaumette [2010]
n_joints=size(q,1);
qd=zeros(size(q));
g=zeros(size(q));
p=0.05;
deltaq=limits(:,2)-limits(:,1);

q_min_0=limits(:,1)+ p*deltaq;
q_max_0=limits(:,2)- p*deltaq;

if(sum(abs(e))~=0)
    Pe=eye(n_joints)-(1/(e'*(J*J')*e))*J'*(e*e')*J;
else
    Pe=eye(n_joints)-invJ*J;
end
% Checks is joint avoidance is required
for i=1:n_joints
    
    if (q(i)<q_min_0(i) || q(i)>q_max_0(i))
        
        if(q(i)<q_min_0(i))
            g(i)=-1; 
            disp(strcat('Min breached',num2str(i)))
        else
            g(i)=1;
            disp(strcat('Max breached',num2str(i)))
        end
    else
        g(i)=0;
    end
end
if sum(abs(g))~=0
    G=diag(g);
    p1=0.5;
    lam=.1;
    
    q_min_1=q_min_0- p1*p*deltaq;
    q_max_1=q_max_0+ p1*p*deltaq;
  
    for i=1:n_joints
        
        if g(i)~=0
            PG=Pe*G(:,i);
            l_sec=0;
            l_i=0;
            l_sec=(1+ lam)*abs(q1(i))/abs(PG(i));
            
            if (q(i)<q_min_1(i) || q(i)>q_max_1(i))
                
                l_i=1;
                
            else
                if(q(i)<q_min_0(i))
                    la_min=1/(1+exp(-12*(q(i)-q_min_0(i))/(q_min_1(i)-q_min_0(i)))+6);
                    la_min0=1/(1+exp(-12*(q_min_0(i)-q_min_0(i))/(q_min_1(i)-q_min_0(i)))+6);
                    la_min1=1/(1+exp(-12*(q_min_1(i)-q_min_0(i))/(q_min_1(i)-q_min_0(i)))+6);
                    l_i= (la_min-la_min0)/(la_min1-la_min0);
                else
                    la_max=1/(1+exp(-12*(q(i)-q_max_0(i))/(q_max_1(i)-q_max_0(i)))+6);
                    la_max0=1/(1+exp(-12*(q_max_0(i)-q_max_0(i))/(q_max_1(i)-q_max_0(i)))+6);
                    la_max1=1/(1+exp(-12*(q_max_1(i)-q_max_0(i))/(q_max_1(i)-q_max_0(i)))+6);
                    l_i= (la_max-la_max0)/(la_max1-la_max0);
                end
            end
            
            qd=qd-l_sec*l_i*PG;
            
        end
    end
end

end
