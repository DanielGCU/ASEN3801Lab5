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
fig_4 = [19, 20, 21, 22, 23, 24];
fig_5 = [25, 26, 27, 28, 29, 30];

PlotOptions = {'r-', 'b--', 'g-', 'm--'};

col_1 = PlotOptions{1};

%% Problem 2

% Case 1 Initial Conditions 
x0_2_1 = zeros(12, 1);
x0_2_1(3) = -1609.34;
x0_2_1(7) = 21; 
control_input_2_1 = zeros(4, 1);

% Run Case 1 using EOM Function 

tspan = linspace(0, 200, 1000);
[t_2_1, states_2_1] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_1, wind_inertial, aircraft_parameters), tspan, x0_2_1);

states_2_1 = states_2_1';

% Plot Case 1 
N = length(t_2_1);
control_2_1 = repmat(control_input_2_1, 1, N);
%PlotAircraftSim(t_2_1, states_2_1, control_2_1, fig_1, col_1);


% Case 2 Initial Conditions 
x0_2_2 = [0, 0, -1800, 0, 0.02780, 0, 20.99, 0, 0.5837, 0, 0, 0];
control_input_2_2 = [0.1079; 0; 0; 0.3182];


% Run Case 2 using EOM Function 
[t_2_2, states_2_2] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_2, wind_inertial, aircraft_parameters), tspan, x0_2_2);

states_2_2 = states_2_2';

% Plot Case 2
N = length(t_2_2);
control_2_2 = repmat(control_input_2_2, 1, N);
%PlotAircraftSim(t_2_2, states_2_2, control_2_2, fig_2, col_1);


% Case 3 Initial Conditions 
x0_2_3 = [0, 0, -1800, deg2rad(15), deg2rad(-12), deg2rad(270), 19, 3, -2, deg2rad(0.08), deg2rad(-0.2), 0];
control_input_2_3 = [deg2rad(5); deg2rad(2); deg2rad(-13); 0.3];


% Run Case 3 using EOM Function

[t_2_3, states_2_3] = ode45(@(t, var) AircraftEOM(t, var, control_input_2_3, wind_inertial, aircraft_parameters), tspan, x0_2_3);

states_2_3 = states_2_3';

% Plot Case 3 

N = length(t_2_3);
control_2_3 = repmat(control_input_2_3, 1, N);
%PlotAircraftSim(t_2_3, states_2_3, control_2_3, fig_3, col_1);


%% Problem 3 

% Initial Conditions (same as 2.2)
x0_3_1 = [0, 0, -1800, 0, 0.02780, 0, 20.99, 0, 0.5837, 0, 0, 0];
control_input_3_1 = [0.1079; 0; 0; 0.3182];
doublet_size = deg2rad(15);
doublet_time = 0.25;

% Run using EOM with time 1  
tspan1 = [0 3]; 

[t_3_1, states_3_1] = ode45(@(t, var) AircraftEOMDoublet(t, var, control_input_3_1, doublet_size, doublet_time, wind_inertial, aircraft_parameters), tspan1, x0_3_1);

states_3_1 = states_3_1';


% Plot 
N = length(t_3_1);
control_3_1 = repmat(control_input_3_1(:), 1, N);
idx1 = t_3_1 > 0 & t_3_1 <= doublet_time;
idx2 = t_3_1 > doublet_time & t_3_1 <= 2*doublet_time;

control_3_1(1, idx1) = control_3_1(1) + doublet_size;
control_3_1(1, idx2) = control_3_1(1) - doublet_size;

PlotAircraftSim(t_3_1, states_3_1, control_3_1, fig_4, col_1);

% Estimate Natural Freq and Damping Ratio


% Run using EOM with time 2 
tspan2 = [0 100]; 

[t_3_2, states_3_2] = ode45(@(t, var) AircraftEOMDoublet(t, var, control_input_3_1, doublet_size, doublet_time, wind_inertial, aircraft_parameters), tspan2, x0_3_1);

states_3_2 = states_3_2';


% Plot 
N = length(t_3_2);
control_3_1 = repmat(control_input_3_1(:), 1, N);
idx1 = t_3_1 > 0 & t_3_1 <= doublet_time;
idx2 = t_3_1 > doublet_time & t_3_1 <= 2*doublet_time;

control_3_1(1, idx1) = control_3_1(1) + doublet_size;
control_3_1(1, idx2) = control_3_1(1) - doublet_size;

% Plot
PlotAircraftSim(t_3_2, states_3_2, control_3_1, fig_5, col_1);

%% Phugoid Damping Ratio Calculation

u = states_3_2(7, :);
time = t_3_2;

% Find peaks
[peak_vals, peak_locs_idx] = findpeaks(u, time);

num_cycles = 1; 

steady_state = 21;
p1 = peak_vals(1) - steady_state;
p2 = peak_vals(1 + num_cycles) - steady_state;

% Logarithmic decrement
delta = (1/num_cycles) * log(p1 / p2);

% Damping ratio
zeta = delta / sqrt(4*pi^2 + delta^2);

% Phugoid period (average time between peaks)
periods = diff(peak_locs_idx);  % time differences between successive peaks
T_phugoid = mean(periods);

% Natural Freq
omega_n = 2*pi / T_phugoid;

fprintf('Phugoid log decrement delta = %.5f\n', delta);
fprintf('Damping ratio zeta = %.5f\n', zeta);
fprintf('Natural frequency omega_n = %.5f rad/s\n', omega_n')











