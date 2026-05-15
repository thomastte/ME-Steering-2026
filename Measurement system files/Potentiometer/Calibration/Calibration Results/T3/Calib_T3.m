clc; clear; close all;

% Folder where the txt files are stored
folder = 'C:\Users\maris\Documents\BACHELOR\Test\Test_potentiometer\Calibration of potentiometer\T3';

% Known angles [deg]
angle_deg = [0 30 60 90 120 150 180 210 240];

% Files for increasing angle
files_up = {'0.txt', '30.txt', '60.txt', '90.txt', ...
            '120.txt', '150.txt', '180.txt', ...
            '210.txt', '240.txt'};

% Files for decreasing angle
files_down = {'0n.txt', '30n.txt', '60n.txt', '90n.txt', ...
              '120n.txt', '150n.txt', '180n.txt', ...
              '210n.txt', '240.txt'};

% Calculate mean ADC value for each file
mean_adc_up = zeros(length(files_up),1);
mean_adc_down = zeros(length(files_down),1);

for i = 1:length(files_up)
    data_up = readmatrix(fullfile(folder, files_up{i}));
    mean_adc_up(i) = mean(data_up);

    data_down = readmatrix(fullfile(folder, files_down{i}));
    mean_adc_down(i) = mean(data_down);
end

% Linear calibration using increasing angle data only
% ADC = a*angle + b
p = polyfit(angle_deg, mean_adc_up', 1);

a = p(1); % sensitivity [ADC counts/deg]
b = p(2); % offset [ADC counts]

% Fitted line
fit_adc = polyval(p, angle_deg);

% R^2 for linear fit
y_bar = mean(mean_adc_up);
SS_tot = sum((mean_adc_up - y_bar).^2);
SS_res = sum((mean_adc_up - fit_adc').^2);
R2 = 1 - SS_res/SS_tot;
fprintf('R^2 = %.8f\n', R2);

% Residuals for increasing data only
residuals_adc = mean_adc_up - fit_adc';
residuals_deg = residuals_adc / a;

RMSE_adc = sqrt(mean(residuals_adc.^2));
RMSE_deg = sqrt(mean(residuals_deg.^2));

max_residual_adc = max(abs(residuals_adc));
max_residual_deg = max(abs(residuals_deg));

% Hysteresis
% Difference between increasing and decreasing measurement at same angle
hysteresis_adc = mean_adc_up - mean_adc_down;
hysteresis_deg = hysteresis_adc / a;

max_hysteresis_adc = max(abs(hysteresis_adc));
max_hysteresis_deg = max(abs(hysteresis_deg));

% Plot calibration result with both increasing and decreasing points
figure;
scatter(angle_deg, mean_adc_up, 'filled');
hold on;
scatter(angle_deg, mean_adc_down, 'filled');
plot(angle_deg, fit_adc, 'LineWidth', 2);
grid on;

xlabel('Angle [deg]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Steering Angle vs ADC output', 'FontSize', 16);
lgd = legend('Increasing angle', 'Decreasing angle', 'Location', 'best');
lgd.FontSize = 14;

% Residual plot for increasing data only
figure;
stem(angle_deg, residuals_adc, 'filled');
grid on;
xlabel('Angle [deg]', 'FontSize', 14);
ylabel('Residual [ADC counts]', 'FontSize', 14);
title('Residual plot', 'FontSize', 16);
yline(0, '--');