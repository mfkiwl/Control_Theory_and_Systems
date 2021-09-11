%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
% Update state
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
if(drones(1).time > command1)
omega = thetadot2omega(thetadot, theta);

% Compute linear and angular accelerations.
a = acceleration(u, theta, xdot, m, g, k, kd);
omegadot = angular_acceleration(u, omega, I, L, b, k);

omega = omega + dt * omegadot;
thetadot = omega2thetadot(omega, theta);
theta = theta + dt * thetadot;
xdot = xdot + dt * a;
x = x + dt * xdot;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Controller - update input
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PIDcontrol;
LogState(:,step)= [x;xdot;theta;omega];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Update output
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
LogInput(:,step) = u;
y = [x;thetadot];
LogOutput(:,step) = y;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% FUNCTIONS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function omega = thetadot2omega(thetadot, theta)
    ph = theta(1);
    th = theta(2);
    M = [
            1,  0,  -sin(th);
            0, cos(ph),  cos(th)*sin(ph);
            0, -sin(ph), cos(th)*cos(ph)
         ];
    omega = M*thetadot;
end

function thetadot = omega2thetadot(omega, theta)
    ph = theta(1);
    th = theta(2);
    M = [
            1,  0,  -sin(th);
            0, cos(ph),  cos(th)*sin(ph);
            0, -sin(ph), cos(th)*cos(ph)
         ];
    thetadot = M\omega;
end

% Compute thrust given current inputs and thrust coefficient.
function T = thrust(inputs, k)
% Inputs are values for ωi
    T = [0; 0; k * sum(inputs)];

end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(inputs, L, b, k)
% Inputs are values for ωi
    tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

%x2 - linear velocity
function a = acceleration(inputs, angles, xdot, m, g, k, kd)
    gravity = [0; 0; -g];
    R = rotation(angles);
%     R = eul2rotm(angles.','ZYZ');
    
    T = R * thrust(inputs, k);
    Fd = -kd * xdot;
    a = gravity + 1 / m * T + Fd;
end

%x4 - angular velocity vector
function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    omegadot = I\(tau - cross(omega, I * omega));
end

function R = rotation(angles)
ph = angles(1);
th = angles(2);
ps = angles(3);
R = [
        cos(ph)*cos(th)*cos(ps)-sin(ph)*sin(ps),  -cos(ps)*sin(ph)-cos(ph)*sin(th)*sin(ps), cos(ph)*sin(th);
        cos(ph)*sin(ps)+cos(th)*cos(ps)*sin(ph),  cos(ph)*cos(ps)-cos(th)*sin(ph)*sin(ps),  sin(ph)*sin(th);
        -cos(ps)*sin(th),                   sin(th)*sin(ps),        cos(th)
    ];

end

