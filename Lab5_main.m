%{
Author(s): Daniel Ghoreyshi, Ridley Avila, Matthew Buccio, Aleksei Suchkov

Course: ASEN 3801 -- Aero Vehicle Dynamics and Controls Lab 5: Fixed-Wing Aircraft Simulation and Control

Goal: Calculate aerodynamic forces and moments from the aircraft state, control surfaces, and
aerodynamic coefficients. Determine modal characteristics from aircraft performance data

%}

%% Housekeeping 

clc
close all

%% Constants and Other

wind_inertial = zeros(3, 1);

fig_1 = [1, 2, 3, 4, 5, 6];
fig_2 = [7, 8, 9, 10, 11, 12];
fig_3 = [13, 14, 15, 16, 17, 18];

PlotOptions = {'r-', 'b--', 'g-', 'm--'};

col_1 = PlotOptions{1};

%% Problem 2

% Case 1 Initial Conditions 
x0_2_1 = zeros(12, 1);
x0_2_1(3) = -1609.34;
x0_2_1(7) = 21; 
control_input_2_1 = zeros(4, 1);

% Run Case 1 using EOM Function 

tspan = linspace(0, 100, 1000);
[t_2_1, states_2_1] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_1, wind_inertial, aircraft_parameters), tspan, x0_2_1);

states_2_1 = states_2_1';

% Plot Case 1 
N = length(t_2_1);
control_2_1 = repmat(control_input_2_1, 1, N);

PlotAircraftSim(t_2_1, states_2_1, control_2_1, fig_1, col_1);


% Case 2 Initial Conditions 
x0_2_2 = [0, 0, -1800, 0, 0.02780, 0, 20.99, 0, 0.5837, 0, 0, 0];
control_input_2_2 = [0.1079; 0; 0; 0.3182];


% Run Case 2 using EOM Function 
[t_2_2, states_2_2] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_2, wind_inertial, aircraft_parameters), tspan, x0_2_2);

states_2_2 = states_2_2';

% Plot Case 2
N = length(t_2_2);
control_2_2 = repmat(control_input_2_2, 1, N);

PlotAircraftSim(t_2_2, states_2_2, control_2_2, fig_2, col_1);


% Case 3 Initial Conditions 
x0_2_3 = [0, 0, -1800, 15*pi/180, -12*pi/180, 270*pi/180, 19, 3, -2, 0.08, -0.2, 0];
control_input_2_3 = [5*pi/180; 2*pi/180; -13*pi/180; 0.3];


% Run Case 3 using EOM Function

[t_2_3, states_2_3] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_3, wind_inertial, aircraft_parameters), tspan, x0_2_3);

states_2_3 = states_2_3';

% Plot Case 3 

N = length(t_2_3);
control_2_3 = repmat(control_input_2_3, 1, N);
PlotAircraftSim(t_2_3, states_2_3, control_2_3, fig_3, col_1);


%% Problem 3 

% Initial Conditions (same as 2.2)
x0_3 = [0, 0, -1800, 0, 0.02780, 0, 20.99, 0, 0.5837, 0, 0, 0];
control_input_3 = [0.1079*180/pi, 0, 0, 0.3182];

% Run using EOM with time 1  


% Plot 


% Run using EOM with time 2 


% Plot








