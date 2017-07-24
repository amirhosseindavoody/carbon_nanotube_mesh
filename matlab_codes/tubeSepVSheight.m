dia = [CNTDiameter(7,5) CNTDiameter(7,6) CNTDiameter(8,6) CNTDiameter(8,7) ...
    CNTDiameter(9,7)];
hel = [helicity(7,5) helicity(7,6) helicity(8,6) helicity(8,7) helicity(9,7)];
curv = (1.49./dia.^2).*(1+9.89./dia.^5.*1000.*cos(6.*hel));

a = linspace(1,10,1000);

t_a = zeros(5,length(a));


figure
hold on
cmap = hsv(5);
for i=1:length(dia)
    for j=1:length(a)
        t_a(i,j) = fp(a(j),dia(i)/2,curv(i),0);
    end
    plot(a,t_a(i,:),'Color',cmap(i,:));
end
set(gca,'Color',[0.8 0.8 0.8]);
legend on
legend('(7,5)','(7,6)','(8,6)','(8,7)','(9,7)');

