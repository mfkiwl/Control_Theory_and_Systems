%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Visualisation code for quadcopter 
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%Define total width, length and height of flight arena (metres)
spaceDim = 10;
spaceLimits = [-spaceDim/2 spaceDim/2 -spaceDim/2 spaceDim/2 0 spaceDim/2];

sim_time = 90;

%do you want to draw a ground image on the figure?
draw_ground = false;
if(draw_ground)
    ground_img = imread('ground.png');
end


%figure to display drone simulation
f1 = figure;
ax1 = gca;
view(ax1, 3);
axis equal;
axis(spaceLimits)
grid ON
grid MINOR 
caxis(ax1, [0 spaceDim]);
hold(ax1,'on')
axis vis3d


% Set num_drones to 2 for Q5e
num_drones = 2;

%instantiate a drone object, input the axis and arena limits
drones = [];

% To enable gaussian noise or wind, put 'true' in last two arguments
drones = [drones Drone(ax1, spaceDim, num_drones,sim_time, [0;0;0], false,false,false)];
% Uncommand second drone for Q5e
drones = [drones Drone(ax1, spaceDim, num_drones,sim_time, [0.6;0;0], true,false,false)];

while(drones(1).time < sim_time)
    %clear axis
    cla(ax1);
    
    %update and draw drones
    for i = 1:num_drones
        update(drones(i), drones(1).pos);
    end
    
    %optionally draw the ground image
    if(draw_ground)
        imagesc([-spaceDim,spaceDim],[-spaceDim,spaceDim],ground_img);
    end
    
    %apply fancy lighting (optional)
    camlight
    
    %update figure
    drawnow
    pause(0.01)
end
