clc; clear; close all;

folder = 'C:\Users\maris\Documents\BACHELOR\Strain test 5';

loading_up = {'0 punkt 4', '1kg 1', '2kg 1', '3kg 1', '4kg 1', '5kg 1', '6kg 1', '7kg 1', '8kg 1', '9kg 1', '10kg 1.1.txt', '11kg 1', '12kg 1', '17kg 1', '27kg 1'};
loading_down = {'27kg 1', '17kg 1n', '12kg 1n', '11kg 1n', '10kg 1n', '9kg 1n', '8kg 1n', '7kg 1n', '6kg 1n', '5kg 1n', '4kg 1n', '3kg 1n', '2kg 1n', '1kg 1n', '0 punkt 5'};

mass_kg = [0 1 2 3 4 5 6 7 8 9 10 11 12 17 27];

forces_N = mass_kg*9.81;

%Beregner gjennomsnittet til målinger og lagrer i en kolonne
means_up = zeros(length(loading_up),1);
for i = 1:length(loading_up)
    data_up = readmatrix(fullfile(folder, loading_up{i}));
    means_up(i) = mean(data_up);
end

means_down = zeros(length(loading_down),1);
for i = 1:length(loading_down)
    data_down = readmatrix(fullfile(folder, loading_down{i}));
    means_down(i) = mean(data_down);
end

means_down_flip = flipud(means_down);

% Lineær fit for å finne sensitivitet
p = polyfit(forces_N, means_up, 1);
a = p(1);   % counts/N
b = p(2);

% Lineær fit for nedgående krefter
p_down = polyfit(forces_N, means_down_flip, 1);
a_down = p_down(1);   % counts/N
b_down = p_down(2);

% Hysterese i counts
hysteresis_counts = means_up - means_down_flip;

% Hysterese i N
hysteresis_N = hysteresis_counts / a;

% Maks hysterese
max_hyst_counts = max(abs(hysteresis_counts));
max_hyst_N = max(abs(hysteresis_N));

% Plot loading og unloading i samme figur
figure;
plot(forces_N, means_up, 'o-', 'LineWidth', 1.5, 'MarkerSize', 7);
hold on;
plot(forces_N, means_down_flip, 's-', 'LineWidth', 1.5, 'MarkerSize', 7);
grid on;
xlabel('Force [N]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Loading and unloading response', 'FontSize', 16);
lgd = legend('Loading', 'Unloading', 'Location', 'best');
lgd.FontSize = 14;

