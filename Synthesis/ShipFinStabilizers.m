% script used for tuning and testing the controllers. 
% the system runs on capital letters parameters A,B,C,D,E,F. 
% the controller uses lowercase parameters b,c,d,e,f


x0 = [deg2rad(10);0];
tf = 60;
on;
run('..\parameters.m')

sat = 20;

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
Fw = 0.1;
omega_w = 1.3;



%[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([1 20000 0.5]),2);
[K_LQR_int, ~, Poles_LQR_int] = lqr(A_int,B_int, diag([100000 300000 4]),0.3);



sim('LQR_ship.slx');

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

Fw = 0.1;
omega_w = 1.3;

%[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([1 0.1 0.001]),1000);

%[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([10 0.1 0.0001]),1);

[K_fbl_lqr, ~, Poles_fbl_lqr] = lqr(A_fbl_int,B_fbl_int, diag([1 20 0.001]),1);

sim('IO_FBL_LQR.slx');


%% Sliding Mode Controller

b = B/A;
c = C/A;
d = D/A;
e = E/A;
f = F/A;

p2 = 10;
p1 = 5;
k = 5;
regularizer = 1e-4;


ts = 4.6*p2/p1
tsSigma = (p1*x0(1) + p2*x0(2))/k


sat = 20;
Fw = 0.05;
omega_w = 0.3;

sim('SMC.slx');
