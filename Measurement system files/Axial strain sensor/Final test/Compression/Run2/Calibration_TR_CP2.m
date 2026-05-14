clc; clear; close all;

folder = 'C:\Users\maris\Documents\BACHELOR\Test\Test_TR_compression2';

files = {'0 punkt', '10kg', '20kg', '30kg', '35kg', '36kg', '37kg', '38kg', '39kg', '40kg', '41kg', '42kg', '43kg'};

mass_kg = [0 10 20 30 35 36 37 38 39 40 41 42 43];
%Konverterer til Newton
forces_N = mass_kg*9.81;

%Beregner gjennomsnittet til målinger og lagrer i en kolonne
means = zeros(length(files),1);
for i = 1:length(files)
    data = readmatrix(fullfile(folder, files{i}));
    means(i) = mean(data);
end

%Lager lineær kalibreringskurve for måledata
p = polyfit(forces_N, means', 1);
fitline = polyval(p, forces_N);

%Beregner residualer som differanse mellom predikerte verdier fra lineær
%linje og måledata og finner root mean square error[RMSE] av residualene
residuals = means - fitline';
RMSE = sqrt(mean(residuals.^2));

%konverterer til Newton for et mer verdifullt resultat og bruker
%sensitiviteten fra kalibreringen
residuals_N = (residuals / p(1));
RMSE_N = sqrt(mean(residuals_N.^2));

figure;
scatter(forces_N, means, 'filled');
hold on;
plot(forces_N, fitline, 'r', 'LineWidth', 2);
grid on;
xlabel('Force [N]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Force vs ADC raw', 'FontSize', 16);

lgd = legend('Measured data', ...
    sprintf('ADC = %.2f*F %+.2f', p(1), p(2)), ...
    'Location', 'northwest');
lgd.FontSize = 14;

figure;
stem(forces_N, residuals, 'filled');
grid on;
xlabel('Force [N]', 'FontSize',14);
ylabel('Residual [counts]', 'FontSize',14);
title('Residual plot', 'FontSize',16);
yline(0,'--');

a = p(1); %sensitivity
b = p(2); %offset

%Lager en tabell for oversikt over påført kraft og estimert kraft
%fra lineær modell 
%Arrangerer forces som kolonne
F = forces_N(:);
F_est = (means - b)/a;

disp(table(F, means, fitline(:), residuals, F_est, ...
    'VariableNames', {'Force','MeasuredSignal','PredictedSignal','Residual','Estimated_Force'}))

fprintf('a = %.10f\n', p(1));
fprintf('b = %.10f\n', p(2));

%Beregner R^2
y_bar = mean(means); %gjennomsnitt av hele datasettet
SS_tot = sum((means - y_bar).^2);
SS_res = sum((means - fitline').^2);
R2 = 1 - SS_res/SS_tot;

format long g
R2