clc; clear; close all;

folder = 'C:\Users\maris\Documents\BACHELOR\Strain test 4';

files = {'0 punkt 1', '1kg 1', '2kg 1', '3 kg 1', '4kg 1', '5kg 1', '6kg 1', '7kg 1', '8kg 1', '9kg 1', '10kg 1', '11kg 1', '12kg 1', '17kg 1', '27kg 1'};

mass_kg = [0 1 2 3 4 5 6 7 8 9 10 11 12 17 27];
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