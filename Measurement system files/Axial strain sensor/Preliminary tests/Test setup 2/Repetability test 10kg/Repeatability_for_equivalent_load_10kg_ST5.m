clc; clear; close all;
%Calculating the difference between repeated loading of 10kg for the
%preliminary strain test setup 2
run1 = readmatrix('10kg 1.txt');
run2 = readmatrix('10kg 2.txt');
run3 = readmatrix('10kg 3.txt');

zero1 = readmatrix('0 punkt 1.txt');
zero2 = readmatrix('0 punkt 2.txt');
zero3 = readmatrix('0 punkt 3.txt');
zero4 = readmatrix('0 punkt 4.txt');

mean1 = mean(run1);
mean2 = mean(run2);
mean3 = mean(run3);

mean_zero1 = mean(zero1);
mean_zero2 = mean(zero2);
mean_zero3 = mean(zero3);
mean_zero4 = mean(zero4);

offset1 = (mean_zero1 + mean_zero2)/2;
offset2 = (mean_zero2 + mean_zero3)/2;
offset3 = (mean_zero3 + mean_zero4)/2;

corr_run1 = mean1 - offset1;
corr_run2 = mean2 - offset2;
corr_run3 = mean3 - offset3;

corrected_means = [corr_run1 corr_run2 corr_run3];
