function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

% Figure(1): Inertial position

figure(fig(1));

% X Inertial Position
subplot(3, 1, 1);
hold on;
plot(time, aircraft_state_array(1,:), col);
xlabel('Time (s)');
ylabel('Position (m)');
title('Inertial Position in x direction vs time')


% Y Inertial Position 
subplot(3, 1, 2)
hold on;
plot(time, aircraft_state_array(2,:), col);
xlabel('Time (s)');
ylabel('Position (m)');
title('Inertial Position in y direction vs time')


% Z Inertial Position
subplot(3, 1, 3)
hold on;
plot(time, aircraft_state_array(3,:),col);
xlabel('Time (s)');
ylabel('Positon (m)');
title('Inertial Position in z direction vs time')



% Figure(2): Euler Angles 

figure(fig(2));

% Phi Angle
subplot(3, 1, 1);
hold on;
plot(time, aircraft_state_array(4,:), col);
xlabel('Time (s)');
ylabel('Phi (rad)');
title('Roll vs Time')



% Theta angle
subplot(3, 1, 2)
hold on;
plot(time, aircraft_state_array(5,:), col);
xlabel('Time (s)');
ylabel('Theta (rad)');
title('Pitch vs Time')



% Psi angle
subplot(3, 1, 3)
hold on;
plot(time, aircraft_state_array(6,:), col);
xlabel('Time (s)');
ylabel('Yaw (rad)');
title('Psi vs Time')



% Figure(3): Inertial Velocity in Body

figure(fig(3));

% uE velocity
subplot(3, 1, 1);
hold on;
plot(time, aircraft_state_array(7,:), col);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Inertial Velocity in x direction (body)')



% vE velocity
subplot(3, 1, 2)
hold on;
plot(time, aircraft_state_array(8,:), col);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Inertial Velocity in y direction (body)')


% wE velocity
subplot(3, 1, 3)
hold on;
plot(time, aircraft_state_array(9,:), col);
xlabel('Time (s)');
ylabel('Velocity (m/s)');
title('Inertial Velocity in z direction (body)')


% Figure(4): Angular Velocity in Body

figure(fig(4));

% p (roll rate)
subplot(3, 1, 1);
hold on;
plot(time, aircraft_state_array(10,:), col);
xlabel('Time (s)');
ylabel('p (rad/s)');
title('Roll Rate vs Time')


% q (pitch rate)
subplot(3, 1, 2)
hold on;
plot(time, aircraft_state_array(11,:), col);
xlabel('Time (s)');
ylabel('q (rad/s)');
title('Pitch rate vs Time')


% r (yaw rate)
subplot(3, 1, 3)
hold on;
plot(time, aircraft_state_array(12,:), col);
xlabel('Time (s)');
ylabel('r (rad/s)');
title('Yaw rate ')


% Figure(5): Control Forces 

figure(fig(5));

% Elevator (deg)
subplot(2, 2, 1)
hold on;
plot(time, rad2deg(control_input_array(1,:)), col);
xlabel('Time (s)');
ylabel('Elevator in degrees');
title('Elevator vs Time')


% Aileron (deg)
subplot(2, 2, 2)
hold on;
plot(time, rad2deg(control_input_array(2,:)), col);
xlabel('Time (s)');
ylabel('Aileron in degrees');
title('Aileron vs Time')


% Rudder (deg)
subplot(2, 2, 3)
hold on;
plot(time, rad2deg(control_input_array(3,:)), col);
xlabel('Time (s)');
ylabel('Rudder in degrees');
title('Rudder vs Time')


% Throttle 
subplot(2, 2, 4)
hold on;
plot(time, control_input_array(4,:), col);
xlabel('Time (s)');
ylabel('Throttle');
title('Throttle vs Time')


% Figure(6): 3D Path 

figure(fig(6));
plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), -aircraft_state_array(3,:), col)
hold on;
xlabel('X Position (m)');
ylabel('Y Position (m)');
zlabel('Z Position (m)');
title('3D Path of the Aircraft');
% Reverse z direction to have upwards be positive
%set(gca, 'ZDir', 'reverse')


end
