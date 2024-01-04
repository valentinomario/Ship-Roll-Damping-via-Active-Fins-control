%close all
% clear
% clc

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

sat = 45;
rate = 90;

% matcont: 

% b = -0.1232
% c = -0.1015
% d = -0.0648
% e = -0.8727
% f = -0.0044

x0 = [deg2rad(20);0];
tf = 60;
controller_on = 1;
%% LQR
% (0 0) linearization

A_lin = [0, 1; (D+E)/A, B/A];
B_lin = [0; F/A]; 
C_lin = [180/pi,0];
D_lin = 0;

% Augmented system with integrator dynamic
A_int = [A_lin  zeros(2,1)
         -C_lin 0];
B_int = [B_lin; -D_lin];
C_int = [C_lin, 0];
D_int = D_lin;

lin_sim = 0;
Fw = 0;
%[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([100 5000 200]),1);
%[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([1 1000 12]),1);
%[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([1 1 1]),1);

[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([1 250000 50]),1.01);

%[K_LQR, ~, Poles_LQR] = lqr(A_lin,B_lin, diag([100 5000]),1);
sim('LQR_ship.slx');
% [K_LQR, ~, Poles_LQR] = lqr(A_lin,B_lin, diag([1 5]), 1e-1);

%% Adaptive control

ts = 5;

Am = [0 1; -16/ts^2, -8/ts];
Bm = [0; 16/ts^2];

Am = [0,1;-1.412, -1.0259];
Bm = [0;1];

%ok omega_w = 1.3
%alpha = 2000;
%beta = 1000;
%gamma = 200;

alpha = 200;
beta = 100;
gamma = 1500;

gain_locking_start = 100;
Fw = 0;
omega_w = 1.3;

P = lyap(Am,eye(2));

Ce = [0 1]*P;

x = 0.4; P = lyap(Am,[1-x,0; 0 x]*eye(2)); Ce = [0,1]*P;
sim('Synthesis/adaptive_control_new.slx')

%% IO FBL
b = B/A;
c = C/A;
d = D/A;
e = E/A;
f = F/A;

A_fbl = [0 1; 0 0];
B_fbl = [0;1];
C_fbl = [180/pi, 0];
D_fbl = 0;

A_fbl_int = [A_fbl  zeros(2,1)
             -C_fbl 0];
B_fbl_int = [B_fbl; -D_fbl];
C_fbl_int = [C_fbl, 0];
D_fbl_int = D_fbl;

controller_on = 1;
Fw = 0;
omega_w = 1.3;
%[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([1 1000 0.005]),60);
[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([1 1000 0.08]),60);


sim('IO_FBL_LQR.slx');

%% Sliding Mode Controller

b = B/A;
c = C/A;
d = D/A;
e = E/A;
f = F/A;

%p1 = 1;
%p2 = 0.1;
%k = 5;

p2 = 2;
p1 = 1;
k = 8;

regularizer = 0.1;

Fw = 0.2;
omega_w = 1.3;

sim('SMC.slx')
