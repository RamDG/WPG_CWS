function [ok,breached]=checkLimits(q,limits)
ok=true
br=1;
for i=1:size(q,1)
    if (q(i)<limits(i,1) || q(i)>limits(i,2))
         ok=false;
        if q(i)<limits(i,1)
           breached(br,1)=i;
           breached (br,2)=1;
           breached (br,3)=limits(i,1);
        else
            breached(br,1)=i;
           breached (br,2)=2;
           breached (br,3)=limits(i,2);
        end
        br=br+1;
    end
end

if ok
    breached=0
end