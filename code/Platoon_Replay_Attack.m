%% MAIN - CACC Platoon Nominal Simulation
% This script coordinates the execution of the three modules:
% 1. Inizializzazione_Platoon.m  (Parameters & state allocation)
% 2. Dinamica_Veicoli.m          (Discrete integration of CACC control)
% 3. Plot_Risultati.m            (Data visualization)

clear; close all; clc;

fprintf('===========================================\n');
fprintf('  CACC VEHICLE PLATOON SIMULATION (Merco et al.)  \n');
fprintf('===========================================\n\n');

%% Step 1: Parameter and Variable Initialization
Inizialize_var;

%% Step 2: Vehicle Dynamics Computation
Dinamic_NoAttack;

%% Step 3: Plot Generation
Plot_Platoon;

%% Step 4: Attack Simulation
Dinamic_RepAttack;

%% Step 5: Plot Attack
Plot_Platoon;
fprintf('\nExecution completed successfully\n');
