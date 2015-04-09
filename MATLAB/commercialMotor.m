%% Commercial Motor Model 
% Brad MacNeil 01/04/2015
% This document details all the parameters that were measured to model the
% motor that was built of EECE 380. The tests used to measure the given 
% parameters are detailed in MotorTesting.m The parameters measured include:
% Armature Resistance, Armature Inductance, Back EMF constant,  
%Motor Parameters 
Rl = 6.6; %[Ohms] Armature Resistance 
Ll = 6/1000; %[mH] Armature Inductance 
Kel = 0.05;  % [V/Wm] Back EMF constant 
Kdl = 0.0463; % [Tm/A] Torque constant 

Jml = .000072; % Not tested 
Bl = .0037; % Ke - Kd does not account for load 

GE = tf([Kdl],[Ll Rl]);
GM = tf([1],[Jml Bl]);
G = feedback(GE * GM, Kel);

controller = pid(5, 0.03, 1);
encoder = tf([1],[1 0]);
linearSystem = feedback(controller * G * encoder, 1);

step(linearSystem)