clc; clear; close all; 

%% Chassis
AR26.Mass = 293; % Total car mass with average driver - kg
AR26.WheelBase = 1525e-3; % Car wheelbase - m
AR26.FrontTrackwidt = 1260e-3 % Front Trackwidth
AR26.CGHeight = 230e-3; % Center of Gravity height - m
AR26.aLength = AR26.WheelBase*0.45; 
AR26.bLength = AR26.WheelBase - AR26.aLength; 
AR26.FrontTrackwidth = 1210e-3; % Front Trackwidth - m
AR26.RearTrackWidth = 1209e-3; % Rear Trackwidth - m


%% Aerodynamics


%% Suspension

%% Steering
AR26.SteeringWheelBreakPointsOutside = [0, 16.7, 30.25, 44.2, 61.4, 78.35, 86];
AR26.SteeringWheelBreakPointsInside = flip(-AR26.SteeringWheelBreakPointsOutside);
AR26.SteeringWheelBreakPoints = [AR26.SteeringWheelBreakPointsInside, AR26.SteeringWheelBreakPointsOutside];
AR26.WheelBreakPointsOutside = [0, 5, 10, 15, 20, 25, 30];
AR26.WheelBreakPointsInside = flip(AR26.WheelBreakPointsOutside);
AR26.WheelBreakPoints = [AR26.WheelBreakPointsInside,AR26.WheelBreakPointsOutside];
figure
plot(AR26.SteeringWheelBreakPointsOutside, AR26.WheelBreakPointsOutside)
hold on
plot(AR26.SteeringWheelBreakPointsInside, AR26.WheelBreakPointsInside)
AR26.RackWidth = 424.42e-3; % Steering Rack width - m
AR26.SteeringArmLength = 78.68e-3; % Length from upright tierod point to steering axis - m
AR26.TieRodLength = 339.4e-3; % Tie Rod length - m
AR26.RackAxisLength = 79.52e-3; % Long. length from steering rack to steering axis point - m
AR26.PinionRadius = 20e-3; % Pinion radius - m
%% Brakes
AR26.FrntBrkBias = 0.75; % Front brake bias - ratio
AR26.RearBrkBias = 1 -AR26.FrntBrkBias; % Rear brake bias - ratio
AR26.MaxBrakePressure = 100e5; % Maximum brake bias - Pa
AR26.PistonDiam = 25e-3; % Piston Diameter - meters
AR26.PistonArea = pi*(AR26.PistonDiam/2)^2; % Piston area - m^2
AR26.NPistonAxle = 4; % number of pistons per axle
AR26.DiscRadius = 0.15; % Brake disc radius - meters

%% Tyres
AR26.ScaleFctrs = ones(27,1);
AR26.Fext = [(293*9.81)/4 (293*9.81)/4 (293*9.81)/4 (293*9.81)/4];
AR26.UnloadedRadius = 0.23; % Unloaded tyre radius - m
AR26.LoadedRaidus = 0.2; % Loaded tyre radius - m
AR26.FrontPressure = 2e5;
AR26.RearPressure = 2.1e5;


%% DriveTrain


%% PowerTrain
AR26.MaxTorque = 31.6; % Max torque per motor



%% Initial Conditions
AR26.Vx0 = 1; % inital x velocity - m/s
AR26.Omega0 = AR26.Vx0/AR26.LoadedRaidus; % Initial wheelspeed - rad/s

