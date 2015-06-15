function [ alpha ] = helicity( n , m )
%HELICITY Calculates the helicity of the nanotube based on the hamada
%parameters.
%   The helicity is the angle between the perimiter vector (chirality
%   vector) and the basis vector a1

a = 1.42; %[Angstroms]
a1 = a*[sqrt(3)/2,1/2];
a2 = a*[sqrt(3)/2,-1/2];
ch = n*a1+m*a2;

alpha = acos(dot(a1,ch)/(norm(a1)*norm(ch)));

end

