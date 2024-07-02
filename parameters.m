U = 15; % Nominal forward speed [knots]
Ixx = 3.4263e6; % Roll inertia [Kg*m^2]
rf = 4.22; % Fin moment arm to center of gravity [m]
Kpdot = -0.674e6; % Added inertia coefficient
Kp = -0.5e6; % Linear damping coefficient
Knl = -0.416e6; % Nonlinear damping coefficient
Kup = -15.5; % Added Linear damping coefficient
KphiUU = -1180; % Added restoring moment coefficAient
Af = 1.7; % Fin area
Cl = 0.046; % Linear lift coefficient [N/deg]
rho = 1025; % Water density [kg/m^2]    
g = 9.81; % Gravity acceleration [m/s^2]
V = 355.88; % Volume displacement [m^3]
GMt = 1; % Transverse Metacentric Height [m]


Kalpha = 0.5*rho*U^2*Af*Cl;
Kphi = rho*g*V*GMt;

A = Ixx-Kpdot;
B = Kp + Kup - 2*Kalpha*rf/U;
C = Knl;
D = KphiUU*U^2;
E = -Kphi;
F = -2*Kalpha;

Fw = 0e-2;
omega_w = 0.3;

sat = 20;

%b = B/A
%c = C/A
%d = D/A
%e = E/A
%f = F/A
% matcont: 

% b = -0.1232
% c = -0.1015
% d = -0.0648
% e = -0.8727
% f = -0.0044