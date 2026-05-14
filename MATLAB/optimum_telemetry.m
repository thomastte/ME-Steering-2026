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
x = germany.xposition; 
y = germany.yposition; 

figure 
plot(x,y)