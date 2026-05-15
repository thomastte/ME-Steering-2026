clc; clear; close all;
%Script for determing the signal variation at a constant load
data = readmatrix('10kg constant.txt');

a = 250.01; %Sensitivity from calibration
fs = 10; %samples per second

% Maks og min
max_val = max(data);
min_val = min(data);

% Peak-to-peak variasjon
ptp_counts = max_val - min_val;
ptp_N = (ptp_counts / a);

% Drift fra start til slutt
drift_counts = data(end) - data(1);
drift_N = (drift_counts / a);


%beregner standardavvik med drift
sigma_counts = std(data);
sigma_N = sigma_counts / a;

t_min = (0:length(data)-1)/fs/60;

%Plot of signal over time
figure;
plot(t_min, data, 'LineWidth', 1);
grid on;
xlim([0 12]);
xlabel('Time [min]', 'FontSize', 14);
ylabel('ADC output', 'FontSize', 14);
title('Signal output at constant load', 'FontSize', 16);