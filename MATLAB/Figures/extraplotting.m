clc; clear; close all; 

m       = 293;          % kg   total mass
L       = 1.540;        % m    wheelbase
a_cg    = 0.853;        % m    CG to front axle
h_cg    = 0.241;        % m    CG height
t_f     = 1.410;        % m    front track (FULL width)
g_SI    = 9.81;         % m/s^2
b_cg    = L - a_cg;     % m    CG to rear axle
ay      = 1.5;          % g

% -- Steering geometry -------------------------------------------------------
e       = 15e-3;        % m    mechanical trail          estimate
L_arm   = 84.55e-3;     % m    steering arm length (LTS.m)
r_pin   = 20e-3;        % m    pinion radius

% -- Tyre (.tir file coefficients) -------------------------------------------
PDY1    =  2.1471;
PDY2    = -0.42791;
PKY1    = -31.419;
PKY2    =  1.5115;
FNOMIN  =  750;         % N
ap      =  28.55e-3;    % m    contact patch half-length


ay_sweep = 0:0.05:2;
SIay = ay_sweep.*g_SI;
Fz_static = (m*g_SI)/2 * b_cg/L
dFz = ((m.*SIay.*h_cg)./t_f) .* b_cg/L;

FR = Fz_static + dFz;
FL = Fz_static - dFz;

FyTotal = m.*SIay*(b_cg/L); 

FzReq_R = FyTotal .* (FR)./(max(FR + FL,1));
FzReq_L = FyTotal .* FL./(max(FR + FL,1));

figure
plot(ay_sweep, FzReq_R,LineWidth=3)
hold on
plot(ay_sweep, FzReq_L,LineWidth=3)
xlabel('Lateral Acceleration(g)',FontSize=15)
ylabel('Required Lateral Force (N)',FontSize=15)
title('$F_{y,req,i}$ by lateral acceleration',Interpreter='latex',FontSize=20,FontName="Calibri")
legend('$F_{y,req,R}$','$F_{y,req,L}$',FontSize = 14, Interpreter='latex')
set(gcf, 'Units', 'centimeters', 'Position', [10, 5, 25, 11]); % Screen size
set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [10, 20, 25, 11]); % Export size
saveas(gcf,'LateralReq','svg')