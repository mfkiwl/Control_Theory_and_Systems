clear all;

m = 10;         % mass
g = 9.8;        % gravity 
b = 1.5;        % friction coefficient
Km = 25;        % constant parameter
dt = 0.1;       % sampling time

Total_time = 50;   % simulation time
%Total_step = Total_time / dt + 1;   % simulation step

u = 0;          % initial input
d = 0;          % initial position
theta = 0;      % initial angle
d_dot = 0;       % initial velocity
d_dot_dot = 0;    % initial acceleration
theta_dot = 0;   % initial angular velocity

error = 0;       % initial error
u = 0;           % initial input
previous_u = -4; 
previous_error = 0;

LogState = zeros(Total_time / dt, 8);  % reference = 0.15 (0 <= t < 8)
LogState(1, :) = [d
                  d_dot
                  d_dot_dot
                  theta
                  theta_dot
                  previous_error
                  error
                  u];

% Define which options are you using below
Input_disturbance = true;     % If you want to check the input disturbance, then change from false to true
Sensor_noise = false;          % If you want to check the sensor noise, then change from false to true 


% Define the trajectory with the range of times
r = zeros(Total_time / dt, 1);

r(1 : 80) = 0.15;     % first range of the trajectory 80 because it divided by dt = 0.1

for t = 81 : 160     % second range of the trajectory
    r(t) = 0.95 - 0.1 * t / 10;
end

r(160 : Total_time / dt) = -0.65;   % third range of the trajectory

if Input_disturbance
    for t = 1 : Total_time / dt
        Input_noise = rand / 10 - 0.05;   % Uniform noise with [-0.05, 0.05] rad
        %r(t) = Input_noise + r(t);
    end  
end


for t = 1 : Total_time / dt
    if Sensor_noise
        Gaussian_noise = normrnd(0, 0.005)   % Gaussain noise with variance 0.005 m
    else
        Gaussian_noise = 0;
    end
    
    error = r(t) + Gaussian_noise - d;     % using the input disturbance
    %u = 0.0136 * error - 0.0123 * previous_error - u;    % Discretisation of the transfer function from q2-c and q2-d
    u = 14.36 * error - 12.99 * previous_error - u;       % Faster step response
    
    
    theta_dot = Km * (u - theta);
    d_dot_dot = g * sin(theta) - b * d_dot / m;
    d_dot = d_dot + d_dot_dot * dt;
    d = d + d_dot * dt;   
    theta = theta + theta_dot * dt;
    theta = min(max(theta, -pi / 2), pi / 2);
    
    LogState(t, :) = [d
                      d_dot
                      d_dot_dot
                      theta
                      theta_dot
                      previous_error
                      error
                      u];
                  
    previous_error = error;
end

% Make the plots for checking the results
subplot(2,1,1);
plot((1 : Total_time / dt) / 10, r, (1 : Total_time / dt) / 10, LogState(:, 1))
legend("Dynamic Reference", "Distance of Input disturbance")

% subplot(2,1,1);
% plot((1 : Total_time / dt) / 10, r, (1 : Total_time / dt) / 10, LogState(:, 1))
% legend("Dynamic Reference", "Distance of Sensor noise")

subplot(2,1,2);
plot((1 : Total_time / dt) / 10, LogState(:,7))
legend("Error")

