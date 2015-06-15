%Just a quick calculator for where to put my boxes in blender
clc;
scale = .25; %length of all sides of cube
format long

%assuming the constraint is in center of cube equidistant between the two
%cubes
angle = 30;
height = 2
h = scale*tan(15/180*pi);

startLoc = height/2;%+.04;

numCubes = 10;

iPos = zeros(1,numCubes);
iPos(1) = startLoc;

for i=2:numCubes
    iPos(i) = startLoc+(height+h)*(i-1);
end
    
iPos
h

