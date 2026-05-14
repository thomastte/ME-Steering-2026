clc; clear;
T_ftlbs = 350; %ft-lbs
%Convert to Nm 
T_m = T_ftlbs * 1.35582; % Convert ft-lbs to Nm
%Convert to Nmm
T = T_m * 1e3; % Convert Nm to Nmm
D = 19.05; %mm
r = D/2; %mm
%Find polar moment of inertia 
J = (pi * r^4) / 2; % Polar moment of inertia for a solid circular shaft
%Find stress from torsion
tau = T * r / J % Calculate shear stress from torsion
