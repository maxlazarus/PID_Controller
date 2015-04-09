%% EECE 380 Motor Model 
% Brad MacNeil 01/04/2015
% This document details all the parameters that were measured to model the
% motor that was built of EECE 380. The tests used to measure the given 
% parameters are detailed in MotorTesting.m The parameters measured include:
% Armature Resistance, Armature Inductance, Back EMF constant,  
%Motor Parameters 
Ra = 21.9; %[Ohms] Armature Resistance 
La = 14/1000; %[mH] Armature Inductance 
Kea = 0.0240;  % [V/Wm] Back EMF constant 
Kda = 0.029; % [Tm/A] Torque constant 
%Kb = 1; % [ ] Gear Stiffness constant ??
Jma = 8.1837e-05;
Ba = 0.000743; 
alpha = 1.08/2; %[mm] Half the total backlash width between motor and disk gear
ig = 256; % [256:1] Transmition ratio of the gear system 
Tf1 = tf(Kda, [La Ra]);
Tf3 = tf(1, [Jma Ba]);

FB = tf(Ke, 1);
G = 0.13;

omega = G * feedback(Tf1 * Tf3, FB);
theta = omega * tf(1,[1 0]);
%% Open Loop Motor 
rlocus(theta)
axis([-20 5 -10 10])
%% Close Loop Controller 
% PID Controller   

s = tf('s');

Gcs = pid(5, 0, 1);
theta_c = feedback(theta*Gcs, 1);

hold off;
figure(1);
%% Whole Uncontrolled system 
rlocus (theta);
%% Zoomed in 
rlocus (theta);
axis([-10 10 -200 200]);

figure(2);
%% Whole Controller system 
pzmap (theta_c);
%% Zoomed in 
pzmap (theta_c);
axis([-100 100 -200 200]);

figure(3)
plotAngularVelocityStep;

hold on;
step(omega * tf([1],[1 0]));
axis([0 1 0 0.2]);

hold off;
figure(4)
% step(theta_c);
hold on;
axis([0 5 0 65]);
plotAngularPositionStep;

