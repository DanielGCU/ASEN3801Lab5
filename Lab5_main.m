%{
Author(s): Daniel Ghoreyshi, Ridley Avila, Matthew Buccio, Aleksei Suchkov

Course: ASEN 3801 -- Aero Vehicle Dynamics and Controls Lab 5: Fixed-Wing Aircraft Simulation and Control

Goal: Calculate aerodynamic forces and moments from the aircraft state, control surfaces, and
aerodynamic coefficients. Determine modal characteristics from aircraft performance data

%}

%% Housekeeping 

clc
clears
close all

%% Problem 2

% Case 1 Initial Conditions 
x0_2_1 = zeros(12, 1);
x0_2_1(3) = -1609.34;
x0_2_1(7) = 21; 
control_input_2_1 = zeros(4, 1);


% Run Case 1 using EOM Function 


% Plot Case 1 


% Case 2 Initial Conditions 
x0_2_2 = [0, 0, -1800, 0, 0.02780, 0, 20.99, 0, 0.5837, 0, 0, 0];
control_input_2_2 = [0.1079*180/pi, 0, 0, 0.3182];


% Run Case 2 using EOM Function 


% Plot Case 2


% Case 3 Initial Conditions 
x0_2_3 = [0, 0, -1800, 15, -12, 270, 19, 3, -2, 0.08, -0.2, 0];
control_input_2_3 = [5, 2, -13, 0.3];


% Run Case 3 using EOM Function


% Plot Case 3 




