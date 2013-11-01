%% JAraujo 28-5-2012
%
% This code is run whenever the simulation file tankmodel.mdl runs in
% simulatink and is the responsible for defining the parameters of the
% system.
%
% A,B,C,D define the model of the tanks
%
% K is the controller value
%
% The control input to the motor is defined as:
%
% u = -K*z where,
%
% z = [x; x_i] and where "x" is the system state and defined as 
% 
% x = [upper_tank_value ; lower_tank_value]; and
%
% x_i = integral(ref - lower_tank_value)
%
% By using "x_i" we are performing integral control, this means that we what
% will happen is that the "lower_tank_value" will be equal to "ref" as time
% goes on. We use two different "ref" values, one in the beginning, and
% then another that is introduced at "time_step".
%
% The water in the tanks starts at "x0" values.
%
% The simulation time is defined by "simulation_time"
%

clc
clear all
close all
%% Select tank parameters (this was performed using real tanks)
nTanks=1;
K = [3.7 5.2];
TAU = [21.5 26.6];
GAMMA = [0.979 0.925];

for i=1:nTanks
    k = K(i);
    tau = TAU(i);
    gamma = GAMMA(i);
end

du = 1;
dx = 7.5-3.8;
k = dx/du;
dx63 = 0.632*1*k;
dx63real = dx63+3.8; % check the time that you get this value in the plot (127)
gamma = 5/3.9;
tau = 115-90;

%% Construct the state-space system discreption of the tank
alfa1=-1/tau;
beta2=gamma/(gamma*tau);
alfa2=-1/(gamma*tau);
beta1=k/tau;

A=[alfa1 0;
    beta2 alfa2];
B=[beta1;0];
C = eye(2);
D = zeros(2,1);

x0 = [0; 0]; % Initial conditions of the water levels

sys = ss(A,B,C,D);

tstep = 0.001; % define simulation step (now is 10ms)
sysd = c2d(sys,tstep);

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%% Construct model with integral so we can use integral control
Ap = [alfa1 0 0;
   beta2 alfa2 0;
   0 -1 0];

Bp = [beta1;0;0];
Cp = eye(3);
Dp = zeros(3,1);

nA = 3;
nB = 1;

% %% Define the controller
% Q = 0.01*eye(nA);
% R = 0.1*eye(nB);
% [K,S,e] = lqr(Ap,Bp,Q,R); % the controller value is K
% 
% K = [1.1437 3.3787 0.3162];
% xi = 0;
% x = zeros(2,1);
% ifin = 100/tstep;
% u = 0;
% ref = 5;
% j = 1;
% for i=1:ifin
%    
%     if j == 200 
%     xi = xi + 0.2*(x(2,1) - ref);
%     u = -K*[(x(1,1));(x(2,1));xi];
%     j = 1;
%     end
%     j = j + 1;
%     x = Ad*x + Bd*u;
%     xlog(:,i) = x;
%     ulog(:,i) = u;
% end
% figure,
% plot(tstep*(1:ifin),xlog)
% hold on
% plot(tstep*(1:ifin),ulog,'r')
