clc; clear; close all;

folder = 'C:\Users\maris\Documents\BACHELOR\Test\Test_Torque_sensor\Test_torque\Test 3 CW';

loading_up = {'0 punkt', '1kg', '2kg', '3kg', '4kg', '5kg', '6kg', '7kg', '8kg', '9kg', '10kg'};
loading_down = {'10kg', '9kg n', '8kg n', '7kg n', '6kg n', '5kg n', '4kg n', '3kg n', '2kg n', '1kg n', '0 punkt 2'};

mass_kg = [0 1 2 3 4 5 6 7 8 9 10];
arm = 0.218; %m
Torque = mass_kg*9.81*arm;

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
p = polyfit(Torque, means_up, 1);
a = p(1);   % counts/N
b = p(2);

% Lineær fit for nedgående krefter
p_down = polyfit(Torque, means_down_flip, 1);
a_down = p_down(1);   % counts/N
b_down = p_down(2);

% Hysterese i counts
hysteresis_counts = means_up - means_down_flip;

% Hysterese i N
hysteresis_Nm = hysteresis_counts / a;

% Maks hysterese
max_hyst_counts = max(abs(hysteresis_counts));
max_hyst_Nm = max(abs(hysteresis_Nm));

% Plot loading og unloading i samme figur
figure;
plot(Torque, means_up, 'o-', 'LineWidth', 1.5, 'MarkerSize', 7);
hold on;
plot(Torque, means_down_flip, 's-', 'LineWidth', 1.5, 'MarkerSize', 7);
grid on;
xlabel('Torque [Nm]', 'FontSize', 14);
ylabel('ADC value', 'FontSize', 14);
title('Loading and unloading response', 'FontSize', 16);
legend('Loading', 'Unloading', 'Location', 'best');