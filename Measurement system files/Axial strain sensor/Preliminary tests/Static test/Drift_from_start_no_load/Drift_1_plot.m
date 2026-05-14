clc; clear; close all;

data = readmatrix('Drift_9_uten_oppvarming.txt');

%a = 241814.62; %Sensitivity from calibration
a = 250.01;
% Maks og min
max_val = max(data);
min_val = min(data);

% Peak-to-peak variasjon
ptp_counts = max_val - min_val;
ptp_N = (ptp_counts / a);

% Drift fra start til slutt
drift_counts = data(end) - data(1);
drift_N = (drift_counts / a);


fs = 10;
t = (0:length(data)-1)/fs;
t_min = t/60;
figure;
plot(t_min, data, 'LineWidth', 1);
grid on;
xlim([0 30]);
xlabel('Time [min]', 'FontSize', 14);
ylabel('ADC output', 'FontSize', 14);
title('Zero load drift over time', 'FontSize', 16);