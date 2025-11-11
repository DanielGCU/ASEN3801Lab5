function xdot = AircraftEOM(~, aircraft_state, aircraft_surfaces, wind_inertial, aircraft_parameters)

%----------------------------------------------------------------
% Inputs:
%   time [s]
%   aircraft_state: 12x1 state vector
%   aircraft_surfaces: 4x1 control surface vector
%   wind_inertial: 3x1 inertial wind velocity in inertial coordinates
%   aircarft_parameters: aircarft parameter structure

% Outputs:
%   xdot: derivative of state vector
%----------------------------------------------------------------

% Extract aircraft state
xE = aircraft_state(1);
yE = aircraft_state(2);
zE = aircraft_state(3);
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);
uE = aircraft_state(7);
vE = aircraft_state(8);
wE = aircraft_state(9);
inert_vel_b = [uE;vE;wE];
p = aircraft_state(10);
q = aircraft_state(11);
r = aircraft_state(12);
ang_vel = [p;q;r];

% Extract aircraft parameters
g = aircraft_parameters.g;
m = aircraft_parameters.m;
Ix = aircraft_parameters.Ix;
Iy = aircraft_parameters.Iy;
Iz = aircraft_parameters.Iz;
Ixz = aircraft_parameters.Ixz;

% Aero Forces and Moments

[aeroforces, aeromoments] = AeroForcesAndMoments(aircraft_state, aircraft_surfaces, wind_inertial, density, aircraft_parameters);

% Unpack aeroforces vector
X = aeroforces(1);
Y = aeroforces(2);
Z = aeroforces(3);

% Unpack aeromoments vector
L = aeromoments(1);
M = aeromoments(2);
N = aeromoments(3);

% Trigoneometric Evaluated Angles
cPH = cos(phi);
cT = cos(theta);
cPS = cos(psi);
sPH = sin(phi);
sT = sin(theta);
sPS = sin(psi);
tT = tan(theta);
secT = sec(theta);

% Rotation Matrix
Rot321 = [cT*cPS,sPH*sT*cPS-cPH*sPS,cPH*sT*cPS+sPH*sPS;
             cT*sPS,sPH*sT*sPS+cPH*cPS,cPH*sT*sPS-sPH*cPS;
             -sT,sPH*cT,cPH*cT];

% Euler Angle Rates Matrix
EulMat = [1,sPH*tT,cPH*tT;
          0,cPH,-sPH
          0,sPH*secT,cPH*secT];

% Inerital Position Derivatives
inerital_pos_dot = Rot321 * [uE;vE;wE];

% Euler Angle Derivatives
euler_angle_rates = EulMat * ang_vel;

% Inertial Velocity in Body Derivatives
uE_dot = r*vE - q*wE - g*sT + X/m;
vE_dot = p*wE - r*uE + g*cT*sPH + Y/m;
wE_dot = q*uE - p*vE + g*cT*cPH + Z/m;

% Roll Rate Derivatives
gamma = Ix*Iz - (Ixz)^2;
gamma_1 = (Ixz * (Ix - Iy + Iz)) / gamma;
gamma_2 = (Iz*(Iz - Iy) + Ixz^2) / gamma;
gamma_3 = Iz / gamma;
gamma_4 = Ixz / gamma;
gamma_5 = (Iz - Ix) / Iy;
gamma_6 = Ixz / Iy;
gamma_7 = (Ix*(Ix - Iy) + Ixz^2) / gamma;
gamma_8 = Ix / gamma;

p_dot = (gamma_1*p*q) - (gamma_2*q*r) + (gamma_3*L) + (gamma_4*N);
q_dot = (gamma_5*p*r) - (gamma_6)*(p^2 - r^2) + M/Iy;
r_dot = (gamma_7*p*q) - (gamma_1*q*r) + (gamma_4*L) + (gamma_8*N);

xdot = [inerital_pos_dot;euler_angle_rates;uE_dot;vE_dot;wE_dot;p_dot;q_dot;r_dot];

end
