function TiTj = dgmgenstructure(ant,gamma,b,alpha,d,theta,r,i,j)
global T_transl
Nf = max(size(ant));

TiTj = sym('TiTj',[4 4]);

% Order antecedents
order=zeros(1,Nf+1);
a = j;
k=1;
while a ~= i
    order(k)=a;
    a=ant(a);
    k = k + 1;
end

TiTj = eye(4);
if i == j
    return
end

% Rounding numerical sin and cos
SG = sin(gamma);
CG = cos(gamma);
SA = sin(alpha);
CA = cos(alpha);
ST = sin(theta);
CT = cos(theta);
for g=1:Nf
    if ~strcmp(class(gamma(g)),'sym')     % Test if variable is symbolic
        SG(g) = round(sin(gamma(g))*10000000)/10000000;
        CG(g) = round(cos(gamma(g))*10000000)/10000000;
    end
    if ~strcmp(class(alpha(g)),'sym')
        SA(g) = round(sin(alpha(g))*10000000)/10000000;
        CA(g) = round(cos(alpha(g))*10000000)/10000000;
    end
    if ~strcmp(class(theta(g)),'sym')
        ST(g) = round(sin(theta(g))*10000000)/10000000;
        CT(g) = round(cos(theta(g))*10000000)/10000000;
    end
end

k = 1;
while order(k) ~= 0
    l = order(k);
    TiTjtemp(1,1) = (CG(l)*CT(l)-SG(l)*CA(l)*ST(l));
    TiTjtemp(2,1) = (SG(l)*CT(l)+CG(l)*CA(l)*ST(l));
    TiTjtemp(3,1) = (SA(l)*ST(l));
    TiTjtemp(4,1) = 0;
    TiTjtemp(1,2) = (-CG(l)*ST(l)-SG(l)*CA(l)*CT(l));
    TiTjtemp(2,2) = (-SG(l)*ST(l)+CG(l)*CA(l)*CT(l));
    TiTjtemp(3,2) = (SA(l)*CT(l));
    TiTjtemp(4,2) = 0;
    TiTjtemp(1,3) = (SG(l)*SA(l));
    TiTjtemp(2,3) = (-CG(l)*SA(l));
    TiTjtemp(3,3) = CA(l);
    TiTjtemp(4,3) = 0;
    TiTjtemp(1,4) = (d(l)*CG(l)+r(l)*SG(l)*SA(l));
    TiTjtemp(2,4) = (d(l)*SG(l)-r(l)*CG(l)*SA(l));
    TiTjtemp(3,4) = (r(l)*CA(l)+b(l));
    TiTjtemp(4,4) = 1;
    TiTj = TiTjtemp*TiTj;
    k = k + 1;
end

if order(k)==0
    TiTj = T_transl*TiTj;
end