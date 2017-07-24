function [ i ] = getNumSec( n, m )
%GETNUMSEC Does what lengthExplore does but in function form

d = CNTDiameter(n,m);
hel = helicity(n,m);
c = critCurv(d,hel);

% want range of [a b] to be able to get any length within an interval [100
% 200] nm. 

% i_min*a + (i_min - 1) * t(a) = l_min = 100
% i_min*(a+t(a)-t(a)/i_min) = l_min
l_min = 100;

a_min = 2;

a = a_min;
range = 1;
steps = 1000;
stepSize = range/(steps);

prevDec = 1;
prevVal = 0;
i = 0;
%t_a increases monotonically with a

% Finds number of tubes at a particular height
while a < a_min+range
    t_a = fp(a,d/2,c,0);
    val = (l_min + t_a)/(a + t_a);
    currDec = val - floor(val);
    if currDec > prevDec
        i = floor(prevVal);
        break;
    end
    prevDec = currDec;
    prevVal = val;
    a = a + stepSize;
end

end

