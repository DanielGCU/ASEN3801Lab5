function TransformFromInertialToBody(wind_inertial, aircraft_state)

% Extract Angles
phi = aircraft_state(4);
theta = aircraft_state(5);
psi = aircraft_state(6);

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

end
