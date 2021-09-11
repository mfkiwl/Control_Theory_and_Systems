% Compute system inputs and updated state.

% Prevent wind-up
if max(abs(params.integral2)) > 0.01
params.integral2(:) = 0;
end

% Compute desire angle
% theta_d = desireAngle();

% Altitude
if(drones(1).time > command1)
h = m*(2.5-x(3))/dt/dt;
e_h = Kd_h*h + Kp_h*params.integral4 + Ki_h*params.integral3;
params.integral3 = params.integral3 + dt .* h;
params.integral4 = params.integral4 + dt .* params.integral3;
end

% Compute total thrust
total = m*g / k / (cos(theta(1)) * cos(theta(2))) + e_h ;

% desireAngle = angleAB(x,trajA,step)-theta;
if(drones(1).time < command1)
    e_angle = thetadot;
else
%     e_angle = (angleAB(x,trajA,t)- theta);
%     t=t+1;
% e_angle = thetadot;
e_angle = [0;0;0.2];
end

% Compute errors
e = Kd * e_angle + Kp * params.integral - Ki * params.integral2;


% Solve for the inputs, γi
u = error2inputs(b, k, L, Ixx, Iyy, Izz, e, total);

% Update the state
params.integral = params.integral + dt .* e_angle;
params.integral2 = params.integral2 + dt .* params.integral;



function inputs = error2inputs(b,k,L,Ixx,Iyy,Izz,e, total)
r1 = -(2*b*e(1)*Ixx + e(3)*Izz*k*L /(4*b*k*L));
r2 = e(3)*Izz/(4*b) - e(2)*Iyy/(2*k*L);
r3 = -(-2*b*e(1)*Ixx+e(3)*Izz*k*L /(4*b*k*L));
r4 = e(3)*Izz/(4*b) + e(2)*Iyy/(2*k*L);
inputs = total/4 + [r1;r2;r3;r4];
end

function e = altitude(state,rz,dt)
ez = rz - state(3);
dez = rz/dt - state(6);

end

% Angle between two point 
function angle = angleAB(x,trajectory,time)
px = x(1);
py = x(2);
rx = trajectory(1,time);
ry = trajectory(2,time);
theta = atan2(rx-px,ry-py);
angle = [0;0;theta];
end