%% MAIN - CACC Platoon Simulation
% This script coordinates the execution of the five modules:
% 1. Inizialize_Platoon.m      (Parameters & state allocation)
% 2. Dinamic_NoAttack.m        (Discrete integration of CACC control)
% 3. Plot_Platoon.m            (Data visualization)
% 4. Dinamic_RepAttack.m       (Simulation of a Replay Attack)
% 5. Detector.m                (Simulation of the Detector based on the attack)   

clear; close all; clc;

fprintf('===========================================================\n');
fprintf(' CACC VEHICLE PLATOON SIMULATION - ATTACK AND DETECTION \n');
fprintf('=========================================================\n\n');

%% Step 1: Parameter and Variable Initialization
Inizialization_Platoon_var;

%% Step 2: Vehicle Dynamics Computation
Dinamic_NoAttack;

%% Step 3: Plot Generation
Plot_Platoon;

%% Step 4: Attack Simulation
Dinamic_ReplayAttack;
Plot_Platoon;

%% Step 5: Detector
Detector;

fprintf('\nExecution completed successfully\n');
