clc; clear; close all;

data = readmatrix('Static_no_load.txt');

a = 28018.97;
% Maks og min
max_val = max(data);
min_val = min(data);

% Peak-to-peak variasjon
ptp_counts = max_val - min_val;
ptp_Nm = (ptp_counts / a);

% Standardavvik
std_counts = std(data);
std_Nm = std_counts / a;

% Drift fra start til slutt. Denne var preget av støy på absolutte første
% og siste måling og derfor ble ne annen metode brukt for å måle drift
%drift_counts = data(end) - data(1);
%drift_Nm = (drift_counts / a);

% Drift based on mean of first and last 10 measurements
N_drift = 10;

start_mean = mean(data(1:N_drift));
end_mean = mean(data(end-N_drift+1:end));

drift_counts = end_mean - start_mean;
drift_Nm = drift_counts / a;

fs = 10;
t = (0:length(data)-1)/fs;
t_min = t/60;
figure;
plot(t_min, data, 'LineWidth', 1);
grid on;
xlim([0 45]);
xlabel('Time [min]', 'FontSize', 14);
ylabel('ADC output', 'FontSize', 14);
title('Signal Stability Under Constant Load', 'FontSize', 16);