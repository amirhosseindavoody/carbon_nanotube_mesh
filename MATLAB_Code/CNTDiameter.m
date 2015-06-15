function [ diameter ] = CNTDiameter( n , m )
%CNTDIAMETER Calculates the diameter of a carbon nanotube based on the 
%   hamada parameters n,m. Return units are Angstroms

a = 1.42*sqrt(3);  %[Angstroms]

diameter = a*sqrt(n^2+m^2+n*m)/pi;

end

