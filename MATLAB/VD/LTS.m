clc; clear; close all; 

%% Chassis
AR26.Mass = 293; % Total car mass with average driver - kg
AR26.WheelBase = 1525e-3; % Car wheelbase - m
AR26.CGHeight = 230e-3; % Center of Gravity height - m
AR26.aLength = AR26.WheelBase*0.45; 
AR26.bLength = AR26.WheelBase - AR26.aLength; 
AR26.FrontTrackwidth = 1210e-3; % Front Trackwidth - m
AR26.RearTrackWidth = 1209e-3; % Rear Trackwidth - m


%% Aerodynamics


%% Suspension

%% Steering

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


