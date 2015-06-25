% criticalCurvature.m
%
% This script will calculate the critical curvature of a nanotube for
% different chiralities. This is based equation 2 in Iijima's paper in '96

clear;
clc;
close all;

dia = [CNTDiameter(7,5) CNTDiameter(7,6) CNTDiameter(8,6) CNTDiameter(8,7) ...
    CNTDiameter(9,7)];
hel = [helicity(7,5) helicity(7,6) helicity(8,6) helicity(8,7) helicity(9,7)];

curv = (1.49./dia.^2).*(1+9.89./dia.^5.*1000.*cos(6.*hel));

%Create a list of chiralities
% nm = linspace(0,10,11);
% pair = zeros(length(nm)*(length(nm)+1)/2,2);
% h = zeros(length(pair),1); %helicity
% dtemp = h; %diameter
% cntr = 1;
% curvCntr = 1;
% for i=0:length(nm)-1
%     for j=0:i
%         pair(cntr,1) = i;
%         pair(cntr,2) = j;
%         h(cntr) = helicity(i,j); %calculate helicity
%         dtemp(cntr) = CNTDiameter(i,j); %calculate diameter
%         if(dtemp(cntr)>= 10 && dtemp(cntr) <= 20)
%             d(curvCntr) = dtemp(cntr);
%             curv(curvCntr) = 1.49/d(curvCntr)^2*(1+9.89/d(curvCntr)^5 ...
%                 *10^3*cos(6*h(cntr)));
%             curvCntr = curvCntr+1;
%         end
%         cntr=cntr+1; 
%     end
% end
% 
% ord = sortrows([d',curv'],1);
% d = ord(:,1)';
% curv = ord(:,2)';
% 
% scatter(d,curv*180/pi,'MarkerEdgeColor',[0 .5 .5],...
%     'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5);
% 
scatter(dia,curv*180/pi,'MarkerEdgeColor',[0 .5 .5],...
    'MarkerFaceColor',[0 .7 .7],'LineWidth',1.5);
title('Curvature vs. Diameter','Fontsize', 18,'interpreter', 'latex');
ylabel('Curvature [$^{\circ}$/\AA]','interpreter', 'latex','Fontsize', 16);
xlabel('Diameter [\AA]','interpreter', 'latex','Fontsize', 16);

