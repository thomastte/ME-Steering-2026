clc; clear; close all;
%Analasys of torque calibration/measurements
folder = 'C:\Users\maris\Documents\BACHELOR\Test\Test_Torque_sensor\Test_torque_SC\Test3CCW';

files = {'0 punkt', '5kg', '6kg', '7kg', '8kg', '9kg', '10kg','11kg','12kg','13kg','14kg','15kg','16kg'};

mass_kg = [0 5 6 7 8 9 10 11 12 13 14 15 16];
arm = 0.27; %meters
%Konverterer til Newton
Torque = mass_kg*9.81*arm; %Nm

%Beregner gjennomsnittet til målinger og lagrer i en kolonne
means = zeros(length(files),1);
for i = 1:length(files)
    data = readmatrix(fullfile(folder, files{i}));
    means(i) = mean(data);
end

%Lager lineær kalibreringskurve for måledata
p = polyfit(Torque, means', 1);
fitline = polyval(p, Torque);

%Beregner residualer som differanse mellom predikerte verdier fra lineær
%linje og måledata og finner root mean square error[RMSE] av residualene
residuals = means - fitline';
RMSE = sqrt(mean(residuals.^2));

%konverterer til Newton for et mer verdifullt resultat og bruker
%sensitiviteten fra kalibreringen
residuals_Nm = (residuals / p(1));
RMSE_Nm = sqrt(mean(residuals_Nm.^2));

figure;
scatter(Torque, means, 'filled');
hold on;
plot(Torque, fitline, 'r', 'LineWidth', 2);
grid on;
xlabel('Torque [Nm]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Torque vs ADC output', 'FontSize', 16);

lgd = legend('Measured data', ...
    sprintf('ADC = %.2f*T %+.2f', p(1), p(2)), ...
    'Location', 'northwest');
lgd.FontSize = 14;

figure;
stem(Torque, residuals, 'filled');
grid on;
xlabel('Torque [Nm]', 'FontSize',14);
ylabel('Residual [counts]', 'FontSize',14);
title('Residual plot', 'FontSize',16);
yline(0,'--');

a = p(1); %sensitivity
b = p(2); %offset

%Lager en tabell for oversikt over påført kraft og estimert kraft
%fra lineær modell 
%Arrangerer forces som kolonne
T = Torque(:);
T_est = (means - b)/a;

disp(table(T, means, fitline(:), residuals, T_est, ...
    'VariableNames', {'Torque','MeasuredSignal','PredictedSignal','Residual','Estimated_Torque'}))

fprintf('a = %.10f\n', p(1));
fprintf('b = %.10f\n', p(2));

%Beregner R^2
y_bar = mean(means); %gjennomsnitt av hele datasettet
SS_tot = sum((means - y_bar).^2);
SS_res = sum((means - fitline').^2);
R2 = 1 - SS_res/SS_tot;

format long g
R2