% ---------------------------------
% Fixed params
% ---------------------------------
g = 9.81; % gravity
m = 0.1;    % mass
L = 0.6;    % Distance from quadcopter center to any of the propellers
b = 0.0002;    % drag coefficient
k = 0.01;    % thrust coefficient
kd = 0.002;   % friction constant
Ixx = 0.005;
Iyy = 0.005;
Izz = Ixx + Iyy;
I = [Ixx,0,0; 0,Iyy,0; 0,0,Izz];   %Inertial matrix
dt = 0.02;

% ---------------------------------
% Linearisation
% ---------------------------------
syms x_ y_ z dx dy dz ph th ps wx wy wz
syms v1 v2 v3 v4
syms dph dth dps

X = [x_;y_;z;dx;dy;dz;ph;th;ps;wx;wy;wz];
U = [v1;v2;v3;v4];
Y = [x_;y_;z;[ph;th;ps]/dt];

% Rotation matrix ZYZ
R = [
        cos(ph)*cos(th)*cos(ps)-sin(ph)*sin(ps),  -cos(ps)*sin(ph)-cos(ph)*sin(th)*sin(ps), cos(ph)*sin(th);
        cos(ph)*sin(ps)+cos(th)*cos(ps)*sin(ph),  cos(ph)*cos(ps)-cos(th)*sin(ph)*sin(ps),  sin(ph)*sin(th);
        -cos(ps)*sin(th),                   sin(th)*sin(ps),        cos(th)
    ];

r = [1,0,-sin(th); 0,cos(ph), cos(th)*sin(ph); 0,-sin(ph),cos(th)*cos(ph)];

% Torques
tph = L*k*(v1-v3);
tth = L*k*(v2 - v4);
tps = b*(v1-v2+v3-v4);
tau = [tph;tth;tps];
omega = [wx;wy;wz];

% New state
dX1 = [dx;dy;dz];
dX2 = [0;0;-g]+ (1/m)*R*[0;0;k*(v1+v2+v3+v4)] + (1/m)*[-kd*dx;-kd*dy;-kd*dz];
dX3 = r \ [wx;wy;wz];
dX4 = I\(tau - cross(omega, I * omega));
dX = [dX1;dX2;dX3;dX4];

% Jacobian
Aj = jacobian(dX,X);
Bj = jacobian(dX,U);
Cj = jacobian(Y,X);
Dj = jacobian(Y,U);

A = double(subs(Aj,[dx,dy,dz,th,ph,ps,wx,wy,wz,v1,v2,v3,v4],[0,0,0,0,0,0,0,0,0,0,0,0,0]));
B = double(subs(Bj,[ph,th,ps],[0,0,0]));
C = double(Cj);
D = double(Dj);

% eigenvalues = zeros(12,1);
% eigenvalues = rand(12,1);
% eigenvalues = [0.05,0.1,0.1,0.05,0.5,0.1,0.01,0.5,0.2,0.1,0.09,0.1];
% cont_sys = ss(A,B,C,D);
% disc_sys = c2d(cont_sys,dt,'zoh');
% K_dis = place(disc_sys.A,disc_sys.B,eigenvalues);
% ---------------------------------
% Initial simulation state
% ---------------------------------
x = zeros(3,1);      %x1
% x = [0;0;2];
xdot = zeros(3,1);   %x2
theta = zeros(3,1);  %x3
omega = zeros(3,1);  %x4

thetadot = zeros(3,1);
% deviation = 100;
% thetadot = deg2rad(2 * deviation * rand(3, 1) - deviation);

% ---------------------------------
% For PID control
% ---------------------------------
% Controller gains, tuned by hand and intuition.
Kd_h = 0;
Kp_h = 0.5;
Ki_h = 0;
e_h = 0;
params.integral3 = 0;
params.integral4 = 0;

Kp = 0.2;
Kd = 1;
Ki = 0.5;
e_angle = zeros(3,1);
params.integral = zeros(3, 1);
params.integral2 = zeros(3, 1);

% ---------------------------------
% Log
% ---------------------------------
% [x;xdot;theta;omega]
LogState = zeros(12,30/dt);
LogInput = zeros(4,30/dt);
LogOutput = zeros(6,30/dt);
step = 1;

%input
u = zeros(4,1);
y = zeros(6,1);
%Time to sent out the command
command1 = 1.0;
command2 = 7.0;

trajectory;
t=1;