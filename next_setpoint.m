
% get the step from caller, give new setpoint given current_pos
function next_setpoint = get_setpoint(mode, current_pos)
    delta = 0.25;
    r = 2.5;
    switch mode
        case 0
            disp("Awaiting first MOVE command.");
            next_setpoint = [0; 0; 0]; %initial pos (x,y,z) = (0,0,0)
            return
        case 1 %move received
            disp("Received first MOVE command.");
            next_setpoint = [0; 0; 2.5];
            return
        case 2
            disp("Awaiting second MOVE command.");
            next_setpoint = [0; 0; 2.5];
            %hovering here
            return
        case 3
            disp("Received second MOVE command.");
            %draw circle
            x = current_pos(1) + delta;
            if x >= 5
                x = 5;
            end
            y = sqrt(r^2 - (x - r)^2);
            next_setpoint = [x;y;2.5];
            return
        case 4
            disp("Flying other half of circle");
            x = current_pos(1) - delta;
            if x <= 0
                x = 0;
            end
            y = sqrt(r^2 - (x - r)^2);
            next_setpoint = [x;-y;2.5];
            return
        case 5
            disp("Landing now")
           
            next_setpoint = [0; 0; 0];
            return
    end
end