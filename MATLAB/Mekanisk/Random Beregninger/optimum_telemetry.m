clc; clear; close all; 
format longG
germany = readtable("ar26germany.csv");

f = figure(Name="Lateral Accel Germany");
plot(germany.elapsedTime,germany.lateralAcceleration,LineWidth=2)
xlabel('Elapsed Time (s)');
ylabel('Lateral Acceleration (m/s^2)');
title('Lateral Acceleration vs. Elapsed Time');
grid on;
%legend('Lateral Acceleration');
saveas(f, 'lateral_acceleration_plot.svg');
exportgraphics(gcf, 'lateral_acceleration_germany.eps')
austria = readtable("ar26austria.csv");

f2 = figure(Name="Lateral Accel Austria");
plot(austria.elapsedTime, austria.lateralAcceleration, LineWidth=2)
xlabel('Elapsed Time (s)');
ylabel('Lateral Acceleration (m/s^2)');
title('Lateral Acceleration vs. Elapsed Time (Austria)');
grid on;
saveas(f2, 'lateral_acceleration_austria_plot.svg');
exportgraphics(gcf, 'lateral_acceleration_austria_plot.eps')

%Finding circuit distances
% Calculate the circuit distances for Germany and Austria
germanyDistance = max(germany.elapsedDistance)
austriaDistance = max(austria.elapsedDistance)
austriaLaps = 22000/austriaDistance;
germanyLaps = 22000/germanyDistance;
austriaTurns = 41; 
germanyTurns = 58; 
totalAustriaTurns = round(austriaLaps * austriaTurns)
totalGermanyTurns = round(germanyLaps * germanyTurns)


%%
clc; close all; 
xGer = germany.xposition; 
yGer = germany.yposition; 

figure 
plot(xGer,yGer)
hold on
plot(xGer(1),yGer(1),'ks','MarkerSize', 14, 'MarkerFaceColor','r','DisplayName',"Start")
legend("","Start")

xAus = austria.xposition; 
yAus = austria.yposition; 
figure 
plot(xAus, yAus)
hold on
plot(xAus(1),yAus(1),'ks','MarkerSize', 14, 'MarkerFaceColor','r','DisplayName',"Start")
legend("","Start")