clear; close all; clc;

SPEED= 100; % kph

WF = 2*65.925 ; % kg

WR = 2*80.575;

L = 1525; % wheelbase mm

WB = L/1000;

A = WB*WR/(WF+WR);

B = WB*WF/(WF+WR);

IZZ = (WF+WR)*A*B; % your car as a dumbell.

ENFB = .70; % Mz compliance steer initial slope deg/100 Nm

ENFC = 40; % Mz compliance steer range (67% of final value)

enf = 0; % initial compliance steer

STEER= 10; % steady state steer angle

SR = 5; % steer ratio

u = SPEED/3.6; % meters per parsec

R = 0; % initial condition for yaw rate

BETA = 0; % initial condition for sideslip

AYG = 0; % initial lateral acceleration

WF2 = WF/2; % Individual wheel loads (kg)

WR2 = WR/2;

dwf = 0; % load transfer state

dwr = 0;

BETA = 0; % initial sideslip

AYG = 0; % initial Ay

R = 0; % initial yaw velocity

rwa = 0; % reference wheel steer angle

dt = .01; % integration interval

swa_sys =tf(1,[.02 1]); % .02 sec time constant for steer input
%figure
%step(swa_sys)

time = 0:dt:2;

swa = STEER*step(swa_sys,time); % Create smoothed step input

rwa = swa/SR; % road wheel angle(s)

LTF = 30; % just some cute numbers to reduce the boredom.

LTR = 20;

Fy4 = [2.9229 0.71248 0.16625 1.5228]; % Lapsim tire model 4 term fy

Mz4 = [-0.034974 0.013087 0.17205 2.6245]; % Lapsim tire model 4 term Mz

ALPHAF=0;

ALPHAR=0;

n = 0; % log the results at each step

for t= time; % a couple of seconds ought to do

n=n+1;

dwf = AYG * LTF; % contrived load transfer distribution

dwr = AYG * LTR;

wlf = WF2 + dwf; % transfer the weight

wrf = WF2 - dwf;

wlr = WR2 + dwr; % try to remember we did this on the front.

wrr = WR2 - dwr;

ALPHAF = BETA + A*R/u + enf - rwa(n); % Front slip angle

ALPHAR = BETA - B*R/u ;

% we even have aligning moments to add up for rigid body effects as

% well as the nonlinear steering compliance

fylf = Pacejka4_Model(Fy4,[ALPHAF,-9.806*wlf]); %Nonlinear tire FY representation

fyrf = Pacejka4_Model(Fy4,[ALPHAF,-9.806*wrf]); %

fylr = Pacejka4_Model(Fy4,[ALPHAR,-9.806*wlr]); %

fyrr = Pacejka4_Model(Fy4,[ALPHAR,-9.806*wrr]); %

nlf = Pacejka4_Model(Mz4,[ALPHAF,-9.806*wlf]); % Nonlinear tire MZ representation

nrf = Pacejka4_Model(Mz4,[ALPHAF,-9.806*wrf]); %

nlr = Pacejka4_Model(Mz4,[ALPHAR,-9.806*wlr]); %

nrr = Pacejka4_Model(Mz4,[ALPHAR,-9.806*wrr]); %

fyf = fylf + fyrf; % sum of front wheel forces

fyr = fylr + fyrr; % sum of rear wheel forces

nf = nlf + nrf ; % sum of front tire aligning moments

% following is a steering compliance model:

enf = -sign(nf).*ENFB*ENFC/100.*log(abs(nf)./ENFC+1)/2; % Steer compliance

% now put it all together.:

yaw_moment= (180/pi)*(A*fyf - B*fyr +nlf +nrf +nlr +nrr); % just for you, Claude

rd = yaw_moment /IZZ;

R = R + rd*dt; % yawrate degrees

betad = (180/pi)*(fyf + fyr)/(WF + WR)/u - R; % Sideslip velocity

BETA = BETA + betad*dt;

AYG = u*(R + betad)/(180/pi)/9.806; % G whizz

% Save the histories.

r(n) = R;

ayg (n)= AYG;

beta(n)= BETA;

slipf(n)=ALPHAF;

slipr(n)=ALPHAR;

end

figure('menubar','none','numbertitle','off','Name','Vehicle Handling 101')

plot(time,swa/swa(end),time,r/r(end),time,ayg/ayg(end),time,beta/beta(end))

l=legend('Steer Angle','Yawrate','Ay','Sideslip');legend boxoff;grid on

set(l,'location','best')

xlabel('Time (seconds)')

ylabel('Normalized by Their Steady State Values')

figure

plot(time,slipf,time,slipr)

grid on

xlabel('Time (seconds')

ylabel('Tire Slip Angles (deg)')

legend('Front','Rear') 