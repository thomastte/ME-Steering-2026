clc; clear; close all;

% Read data from txt file
data = readmatrix('drift.txt');

% read colums from .txt
time = data(:,1);      % Time,s
raw = data(:,2);       % Raw ADC value
filtered = data(:,3);  % EMA filtered ADC value


% Sensitivity
sensitivity = 12.03;   % ADC counts/deg

% Standard deviation in ADC counts
std_raw_adc = std(raw);
std_filtered_adc = std(filtered);

% Standard deviation converted to degrees
std_raw_deg = std_raw_adc / sensitivity;
std_filtered_deg = std_filtered_adc / sensitivity;

% Peak-to-peak variation in ADC counts
ptp_raw_adc = max(raw) - min(raw);
ptp_filtered_adc = max(filtered) - min(filtered);

% Peak-to-peak variation converted to degrees
ptp_raw_deg = ptp_raw_adc / sensitivity;
ptp_filtered_deg = ptp_filtered_adc / sensitivity;

% Plot
figure;
plot(time, raw);
hold on;
plot(time, filtered, 'LineWidth', 1.5);
grid on;

xlabel('Time [s]');
ylabel('ADC value');
title('Potentiometer signal over time');
legend('Raw signal', 'EMA filtered signal');