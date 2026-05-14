clc; clear; close all;
%plotting the test of +-4KN static load to compare values
run1 = readmatrix('4KN.txt');
run2 = readmatrix('4KN nr 2.txt');
run3 = readmatrix('4KN nr 3.txt');

mean1 = mean(run1);
mean2 = mean(run2);
mean3 = mean(run3);

adc_means = [mean1 mean2 mean3];

format long g

figure;
plot(1:3, adc_means, 'o', 'MarkerSize', 8, 'LineWidth', 1.5);
grid on;
xlabel('Run', 'FontSize', 14);
ylabel('ADC output at 4 kN', 'FontSize', 14);
title('Repeatability at 4 kN', 'FontSize', 16);
xticks([1 2 3]);
xticklabels({'Run 1','Run 2','Run 3'});
