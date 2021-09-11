% Q5a

trajA = trajectoryA(0.02);

function trajectory = trajectoryA(dt)
r2 = 2.5^2;
d = 2.5*2;
frames = 10/dt;
% Half circle
xpf = d/(frames/2);
% Desire [position,linear velocity,acceleration]
trajectory = zeros(9,frames);
% (x-2.5)^2 + (y)^2 = r2
x = 0;
for i=1:frames/2
    y = sqrt(r2-(x-2.5)^2);
    trajectory(1:3,i) = [x;y;2.5];
    trajectory(1:3,frames-i+1) = [x;-y;2.5];
    x = x + xpf;
end

for i=2:frames-1
    trajectory(4:6,i) = trajectory(1:3,i)-trajectory(1:3,i-1);
end
trajectory(4:6,2:end) = abs(trajectory(4:6,2:end)/dt);
trajectory(7:9,2:end) = trajectory(4:6,2:end)/dt;


end
