%Plotting potentiometer ADC values from Arduino IDE test
%Converting to degrees and setting an offset
%Create plot for raw and filtered values

data = readmatrix("250Hz_8Hz_dyna.txt");

raw_adc = data(:,1);
filtered_adc = data(:,2);

%converting to degrees
theta_raw  = (raw_adc - 765.87) ./ 12.03;
theta_filt = (filtered_adc - 765.87) ./ 12.03;

fs = 250;   % Sampling frequnecy
N  = length(theta_raw);

t = (0:N-1)/fs;

% Select time interval and make it start at 0 s
idx = t >= 56 & t <= 57;
t_zoom = t(idx) - 56;

figure
plot(t_zoom, theta_raw(idx),  'r', 'LineWidth', 1.2)
hold on
plot(t_zoom, theta_filt(idx), 'b', 'LineWidth', 1.2)

xlim([0 1]);

xlabel("Time [s]")
ylabel("Steering angle [deg]")
title("Unfiltered and EMA filtered steering angle")
lgd = legend("Unfiltered","EMA filtered");
lgd.FontSize = 14;
grid on