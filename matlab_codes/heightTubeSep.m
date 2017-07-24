clear
close all

dia = [CNTDiameter(7,5) CNTDiameter(7,6) CNTDiameter(8,6) CNTDiameter(8,7) ...
    CNTDiameter(9,7)];

hel = [helicity(7,5) helicity(7,6) helicity(8,6) helicity(8,7) helicity(9,7)];
curv = (1.49./dia.^2).*(1+9.89./dia.^5.*1000.*cos(6.*hel));

t = linspace(-10,10, 10000);
tspace = t(2)-t(1);

h = zeros(5,length(t));
hnorm = zeros(5,length(t));
gofx = zeros(5,length(t));

cmap = hsv(6);

h1 = 1;

figure
hold on
for i=1:length(dia)
    h(i,:) = atan(t/(dia(i)/2))/curv(i) - t/2;
    hnorm(i,:) = t./h(i,:);
    gofx(i,:) = t - (atan(t./(dia(i)/2))./curv(i) - t./2 - h1)./ ...
        ((1/curv(i)).*((dia(i)/2)./(t.^2 + (dia(i)/2)^2)) - .5);
    plot(t, h(i,:),'Color',cmap(i,:));
end
set(gca,'Color',[0.8 0.8 0.8]);
legend on
legend('(7,5)','(7,6)','(8,6)','(8,7)','(9,7)');

x = figure;
hold on
for i=1:length(dia)
    x = plot(t,hnorm(i,:),'Color',cmap(i,:));
end
set(gca,'Color',[0.8 0.8 0.8]);
legend on
legend('(7,5)','(7,6)','(8,6)','(8,7)','(9,7)');

y = figure;
dgofx = zeros(5,length(t)-1);
hold on
for i=1:length(dia)
    y = plot(t,gofx(i,:),'Color',cmap(i,:));
    dgofx(i,:) = diff(gofx(i,:))/tspace;
    
end
y = plot(t,t,'k');
set(gca,'Color',[0.8 0.8 0.8]);
legend on
legend('(7,5)','(7,6)','(8,6)','(8,7)','(9,7)');

z = figure;
hold on;
for i=1:length(dia)
    z = plot(t(1:length(dgofx)),dgofx(i,:),'Color',cmap(i,:));
end
set(gca,'Color',[0.8 0.8 0.8]);
legend on
legend('(7,5)','(7,6)','(8,6)','(8,7)','(9,7)');
