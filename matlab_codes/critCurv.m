function [ c ] = critCurv( d, h )
%CRITCURV Gets critical curvature of nanotube

c = 1.49/(d^2) * (1 + (9.89/d^5)*1000*cos(6*h));
end

