clc; clear; close all; 

a = [55, 52, 60, 58, 63];
b = [152, 156, 154, 152, 155];

avgA = mean(a)
stdA = std(a)
avgB = mean(b)
stdB = std(b)


a = round([a, normrnd(avgA, stdA, [1, 5])]);
newA = mean(a)
b = round([b, normrnd(avgB, stdB, [1, 5])]);
newB = mean(b)