clc; clear; close all;

folder = 'C:\Users\maris\Documents\BACHELOR\Strain_test';
%'-0.25KN nr3.txt','-0.5KN nr3.txt','-0.75KN nr3.txt',
files = {'-0.25KN nr3.txt', '-0.5KN nr3.txt','-0.75KN nr3.txt','-1.0KN nr3.txt','-1.5KN nr3.txt', ...
         '-2.0KN nr3.txt','-2.5KN nr3.txt','-3.0KN nr3.txt','-3.5KN nr3.txt','-4KN nr3.txt'};

forces_kN = [0.25 0.5 0.75 1 1.5 2 2.5 3 3.5 4];

%Beregner gjennomsnittet til målinger
means = zeros(length(files),1);


for i = 1:length(files)
    data = readmatrix(fullfile(folder, files{i}));
    means(i) = mean(data);
end

p = polyfit(forces_kN, means', 1);
fitline = polyval(p, forces_kN);

residuals = means - fitline';
RMSE = sqrt(mean(residuals.^2));

residuals_N = (residuals / p(1))* 1000;
RMSE_N = sqrt(mean(residuals_N.^2));

figure;
scatter(forces_kN, means, 'filled');
hold on;
plot(forces_kN, fitline, 'r', 'LineWidth', 2);
grid on;
xlabel('Compressive Force [kN]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Force vs ADC raw', 'FontSize', 16);

lgd = legend('Measured data', ...
    sprintf('ADC = %.2f*F %+.2f', p(1), p(2)), ...
    'Location', 'northwest');
lgd.FontSize = 14;

figure;
stem(forces_kN, residuals, 'filled');
grid on;
xlabel('Compressive Force [kN]', 'FontSize',14);
ylabel('Residual [counts]', 'FontSize',14);
title('Residual plot', 'FontSize',16);
yline(0,'--');

%Table for residuals
a = p(1);
b = p(2);

F = forces_kN(:);
S_means = means(:);

S_pred = a*F + b;
F_est = (S_means - b)/a;
Residual = S_means - S_pred;

disp(table(F, S_means, S_pred, Residual, F_est, ...
    'VariableNames', {'Force_kN','MeasuredSignal','PredictedSignal','Residual','EstimatedForce_kN'}))

fprintf('a = %.10f\n', p(1));
fprintf('b = %.10f\n', p(2));

%Beregner R^2
y_bar = mean(means); %gjennomsnitt av hele datasettet
SS_tot = sum((means - y_bar).^2);
SS_res = sum((means - fitline').^2);
R2 = 1 - SS_res/SS_tot;

format long g
R2