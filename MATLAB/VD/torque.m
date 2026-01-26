clc; close all; clear; 
%Optimum Lap Power Model
RPM = 0.1:1:20e3;
kW = 80; 

T = min(31.6*4, (kW*9550)./RPM);

plot(RPM,T)

%Steering Angle
data = readtable("yawrate.txt");
%data = table2struct(data);
distance = readtable("distance.txt");
distance = distance.Var1; %Travelled distance in meters
time = readtable("time.txt");
time = time.Var1;
wb = 1.54; %Wheelbase in meters
steering_ratio = 25; % 25:1 steering ratio

v = readtable("velocity.txt"); %velocity in km/h
v = v.Var1./3.6; %velocity in m/s

y = data.Var1; %yaw rate in RPM
%y = y.*(pi/180); %yaw rate in rad/s
y = deg2rad(y); 
steering_angle = atan(wb.*y./v); %steering wheel angle in radians
steering_angle = rad2deg(steering_angle); %steering wheel angle in degrees
steering_speed = diff(steering_angle)./diff(time(1:length(steering_angle))); %steering velocity in deg/s
speed = steering_speed/360 * 60; %steering wheel speed in RPM
steering_speed(end+1) = steering_speed(end);
maks = max(steering_speed); %deg/s
linear = maks*0.33 %deg/s * mm/deg

wheel_angle = steering_angle.*steering_ratio; %steering wheel angle with nonzero ratio
speed2 = (diff(wheel_angle)./diff(time(1:length(wheel_angle))));%/360 * 60; %steer speed in RPM
speed2(end+1) = speed2(end); % in deg/s
peak = max(speed2);
linear2 = peak*0.33
linear2/10

%Plot without steering ratio
figure
subplot(3,1,1)
plot(distance,v)
legend('velocity')
grid on
subplot(3,1,2)
plot(distance,steering_angle)
legend('Steering Angle')
grid on
subplot(3,1,3)
plot(distance, steering_speed)
legend('Steering Speed in RPM')


%Plot with steering ratio
figure
subplot(2,1,1)
plot(distance,wheel_angle)
subplot(2,1,2)
plot(distance,speed2)



%% 
clc; clear; close all; 

Fz = 790; %Normal Force, N
a = 55.667; %short side of contact patch, mm
b = 154; %long side of contact patch, mm
x = linspace(-a/2, a/2, 1000);
y = linspace(-b/2, b/2, 1000);
KPI = 7.98; %sigma, KPI in degrees
caster = 6.18; %tau, caster angle in degrees

p = (6*Fz)/(a^3*b) .* (a/2 - x) .* (a/2 + x); %distribution pressure

plot(p,x)

%xlim([-3000 3000])
grid on

% e0
x0 = -a/2;
x1 = a/2;
y0 = -b/2; 
y1 = b/2; 

e0 = 22.36; %mm
beta = 10; %degrees

ex = e0*cosd(beta);
ey = e0*sind(beta);
rootpart = sqrt((ex - x).^2 + (ey - y).^2);
mu = 1.75; %friction coeff

press = @(x) (6*Fz)/(a^3*b) .* (a/2 - x) .* (a/2 + x);

figure
fplot(press)
xlim([-30 30])

rooty = @(x,y) sqrt((ex - x).^2 + (ey - y).^2);
fun = @(x,y) ((6*Fz)/(a^3*b) .* (a/2 - x) .* (a/2 + x)).*(sqrt((ex - x).^2 + (ey - y).^2));

Mzft = integral2(fun, x0, x1, y0, y1);

Mzf = Mzft*cosd(KPI)*cosd(caster); %Nmm
Mzf = Mzf/1000; %Nm
Mfo = 0.4; %Nm

Lk = 85; %mm
theta_tk = linspace(84.17, 132.55,1000);
theta_rt = linspace(177.21,179.95,1000); %degrees
theta_kk = 68.54; %degrees
Frt = 10; %N
R = 10; %mm
L = Lk .* sind(theta_tk);
Ftt = Frt .* cosd(pi-theta_rt);
i = (Ftt .* L .* cosd(theta_kk-(pi/2)))/(Frt .* R);

Mz = (Mzf+Mzg)/i + Mfo