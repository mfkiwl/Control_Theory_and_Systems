%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%  Code modified by Francisco Vasconcelos from
%%%%
%%%%  Drone class, feel free to add functionality as you see fit
%%%%  Author: Daniel Butters
%%%%  Date: 16/11/17
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef Drone < handle
    properties (Constant)
        %width, length, height offset between centre and rotors
        body = [0.6 0.6 0.0];
        
        %time interval for simulation (seconds)
        dt = 0.02;
        
        % size of floating window that follows drone
        axis_size = 2.;
        
        %colours of each component of drone model
        colours = [[.8 .3 .1];[.2 .2 .5];[.8 .1 .3];[.9 .6 .8];[.9 .2 .4]];
        
        %Follows the drone within the figure
        %Don't use if you are simulating more than one drone!
        %Switch to false to see the overall world view
        drone_follow = false;
        
        g = 9.81; % gravity
%         m = 0.5;    % mass
        m = 0.3;
        L = 0.5;    % Distance from quadcopter center to any of the propellers
        b = 0.002;    % drag coefficient
        k = 0.0000004;    % thrust coefficient
        kd = 0.5;   % friction constant
        Ixx = 0.005;
        Iyy = 0.005;
        Izz = 0.01; %Izz = Ixx + Iyy;
%         I = [Ixx,0,0; 0,Iyy,0; 0,0,Izz];   %Inertial matrix
        I = [0.005,0,0; 0,0.005,0; 0,0,0.01];
        
    end
    properties
        %axis to draw on
        axis
        
        %length of one side of the flight arena
        spaceDim
        
        %limits of flight arena
        spaceLimits
        
        %drone position
        pos
        
        %drone rotation matrix
        R
        
        %Simulation time
        time
        
        %parameter to start drone in random position
        pos_offset
        
        %number of drones
        num_drones
        
        %State
        theta
        thetadot
        omega
        omegadot
        x
        xdot
        a
        iteration
        mode
        
        %input 
        u
        
        %output
        y
        integral
        
        %Log
        state
        error
        LogOutput
        
        % Gaussian noise
        noise_enable 
        LogNoise
        
        % Wind model
        wind_enable 
        wc
        Len
        Int
        wind
        Kwind
        LogWind
        
        isFollow
    end
    methods(Static)
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % DYNAMICS HELPER FUNCTIONS
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


        %CHECK
        function R = rotation(angles)
            ph = angles(1);
            th = angles(2);
            ps = angles(3);
            R = [
                    cos(ps)*cos(th), cos(ps)*sin(ph)*sin(th)-cos(ph)*sin(ps), sin(ph)*sin(ps)+cos(ph)*cos(ps)*sin(th);
                    cos(th)*sin(ps), cos(ps)*cos(ph) + sin(ph)*sin(th)*sin(ps), cos(ph)*sin(th)*sin(ps)-cos(ps)*sin(ph);
                    -sin(th),    cos(th)*sin(ph),    cos(ph)*cos(th)
                ];

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

        %x4 - angular velocity vector
        function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
            tau = Drone.torques(inputs, L, b, k);
            omegadot = I\(tau - cross(omega, I * omega));
        end

        % Compute thrust given current inputs and thrust coefficient.
        function T = thrust(inputs, k)
        % Inputs are values for ωi
            T = [0; 0; k * sum(inputs)];

        end
        
        %CHECK
        %x2 - linear velocity
        function a = acceleration(inputs, angles, xdot, m, g, k, kd)
            gravity = [0; 0; -g];
            R = Drone.rotation(angles);
        %     R = eul2rotm(angles.','ZYZ');

            T = R * Drone.thrust(inputs, k);
            Fd = -kd * xdot;
            %CHECK
            a = gravity + 1 / m * T + 1 / m * Fd;
        end
        
        % Calculate windStrength
        function windNext = windStrength(wind,Len,dt,Int)
            V = 2; % Airspeed of  platform
            noise = rand(3,1).*(Int.^2);
            windNext = (1-V*dt./Len).*wind + sqrt(2*V*dt./Len).*noise;
        end
        
        % Calculate params for wind 
        function model = WindModel()
            % Mean wind field
            wind_u = -2.5+rand*5;
            wind_v = -2.5+rand*5;
            wind_altitude = rand*3.28084; % feet from the ground
            magnitude = sqrt(wind_u^2 + wind_v^2); 
            platform_z = -rand*5;
            % Navigation-frame wind vector
            wc = [magnitude*log(-platform_z/0.15)/log(wind_altitude/0.15) ,0,0].';

            % Turbulence
            % Turbulence scale length
            Lw = -platform_z;
            Lu = -platform_z/(0.177+0.00823)^1.2;
            Lv = Lu;
            Len = [Lu;Lv;Lw];
            % Intensities
            Ow = 0.1*magnitude;
            Ou = Ow/(0.177+0.00823)^0.4;
            Ov = Ou;
            Int = [Ou;Ov;Ow];
            
            model = [Len;Int;wc];
        end
    end
    methods
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %INSTANTIATION OF CLASS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function obj = Drone(axis, spaceDim, num_drones,sim_time, pos,isFollow,noise, wind)
            if nargin > 1
                obj.axis = axis;
                
                obj.spaceDim = spaceDim;
                
                obj.spaceLimits = [(-spaceDim/2)+10 (spaceDim/2)-10 (-spaceDim/2)+10 (spaceDim/2)-10 10 spaceDim-10];
                
                obj.pos = pos;
                
%                 obj.pos_offset = [5.*(rand - 0.5),5.*(rand - 0.5),2.5.*(rand)];
                
                obj.R = [1,0,0;0,1,0;0,0,1];
                
                obj.time = 0;
                
                obj.num_drones = num_drones;
                
                obj.isFollow = isFollow;

                obj.x = pos;      %x1
                obj.xdot = zeros(3,1);   %x2
                obj.theta = zeros(3,1);  %x3
                obj.omega = zeros(3,1);  %x4

                obj.thetadot = zeros(3,1);
     
                obj.iteration = 1;
                

                obj.u = zeros(4,1);
                obj.y = zeros(6,1);
                obj.integral = zeros(3,1);
                obj.state = zeros(12,sim_time/obj.dt);
                
                obj.error = zeros(7, sim_time/obj.dt+1); %[x y z ph th ps thrust] error
                obj.LogOutput = zeros(6,sim_time/obj.dt+1);
                obj.mode = 0; 
                %0: wait first move command, 1: received first move command, 2: wait 2nd move command, 3: received second move command, 4: flying other half of circle, 5: landing
                %3,4,5 auto transition based on checkpoint
                % 6 cut power - power down
                
                if(noise)
                obj.noise_enable = noise;
                obj.LogNoise = zeros(3,sim_time/obj.dt);
                end
                
                if(wind)
                    obj.LogWind = zeros(6,sim_time/obj.dt); %
                    obj.wind_enable = true;
                    windM = obj.WindModel();
                    obj.Len = windM(1:3);
                    obj.Int = windM(4:6);
                    obj.wc = windM(7);
                    obj.Kwind = rand(3,1)*0.001;
                    obj.wind = zeros(3,1);
                end
                
            else
                error('Drone not initialised correctly')
            end
            
        end
        

        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %DRAWING OF DRONE TO FIGURE
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function draw(obj)
            %how big should the moving window be
            cL = obj.axis_size;
            
            %set to false if you want to see world view
            %if(obj.drone_follow)
            %    axis([obj.pos(1)-cL obj.pos(1)+cL obj.pos(2)-cL obj.pos(2)+cL obj.pos(3)-cL obj.pos(3)+cL]);
            %end
            
            %create middle sphere
            [X Y Z] = sphere(8);
            %[X Y Z] = (obj.body(1)/5.).*[X Y Z];
            X = (obj.body(1)/5.).*X + obj.pos(1);
            Y = (obj.body(1)/5.).*Y + obj.pos(2);
            Z = (obj.body(1)/5.).*Z + obj.pos(3);
            s = surf(obj.axis,X,Y,Z);
            set(s,'edgecolor','none','facecolor',obj.colours(1,:));
            
            %create side spheres
            %front, right, back, left
            hOff = obj.body(3)/2;
            Lx = obj.body(1)/2;
            Ly = obj.body(2)/2;
            rotorsPosBody = [...
                0    Ly    0    -Ly;
                Lx    0    -Lx   0;
                hOff hOff hOff hOff];
            rotorsPosInertial = zeros(3,4);
            for i = 1:4
                rotorPosBody = rotorsPosBody(:,i);
                rotorsPosInertial(:,i) = bodyToInertial(obj,rotorPosBody);
                [X Y Z] = sphere(8);
                X = (obj.body(1)/8.).*X + obj.pos(1) + rotorsPosInertial(1,i);
                Y = (obj.body(1)/8.).*Y + obj.pos(2) + rotorsPosInertial(2,i);
                Z = (obj.body(1)/8.).*Z + obj.pos(3) + rotorsPosInertial(3,i);
                s = surf(obj.axis,X,Y,Z);
                set(s,'edgecolor','none','facecolor',obj.colours(i+1,:));
            end
            obj.axis.Title.String = ['Sim Time = ',num2str(obj.time,'%f'),' seconds'];
        end
        
        function vectorInertial = bodyToInertial(obj, vectorBody)
            vectorInertial = obj.R*vectorBody;
        end
        

        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %CONTROLLER FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        function u =  pid_controller(obj, setpoint)

%             Kp_x = 0.01;
            Kp_x = 0.1;
            Kd_x = 0;
            
            Kp_y = Kp_x;
            Kd_y = Kd_x;
            
%             Kp_z = 2.5e5;
            Kp_z = 2.5e6;
%             Kd_z = -3.55e3;
            Kd_z = -3.3e4;

            con_pos = [
                Kp_x,Kd_x;
                Kp_y,Kd_y;
                Kp_z,Kd_z;];
            
%             Kp_ph = 120;
%             Kd_ph = -20;
            Kp_ph = 16;
            Kd_ph = -9;
            
            Kp_th = Kp_ph;
            Kd_th = Kd_ph;
            
%             Kp_ps = 4;
%             Kd_ps = -4;
            Kp_ps = 50;
            Kd_ps = -50;
            
            con_angle = [
                Kp_ph,Kd_ph;
                Kp_th,Kd_th;
                Kp_ps,Kd_ps;];
            
            % Position
            e_pos = (setpoint -obj.y(1:3)).*[1;-1;1];
            prev_error = obj.error(:,obj.iteration);
            e_dot_pos = (e_pos - prev_error(1:3)).*[1;-1;1]/obj.dt;
            
            % Angle
            pre_target_angle = con_pos(:,1).*e_pos + con_pos(:,2).*e_dot_pos;
            AddThrust = pre_target_angle(3);
            target_angle = [pre_target_angle(2);pre_target_angle(1);0];
            target_angle = min(max(target_angle,-pi/4),pi/4);

            angle = obj.integral(1:3);   
            e_angle = target_angle - angle;
            e_dot_angle = (e_angle - prev_error(4:6))/obj.dt;
            
            e = con_angle(:,1).*-e_angle + con_angle(:,2).*e_dot_angle;
            
            thrust =  obj.m*obj.g / obj.k / (cos(angle(1)) * cos(angle(2))) + AddThrust;
            thrust = max(thrust , 0);
            
            % Error to input
            r1 = -((2*obj.b*e(1)*obj.Ixx + e(3)*obj.Izz*obj.k*obj.L) /(4*obj.b*obj.k*obj.L));
            r2 = e(3)*obj.Izz/(4*obj.b) - e(2)*obj.Iyy/(2*obj.k*obj.L);
            r3 = -((-2*obj.b*e(1)*obj.Ixx + e(3)*obj.Izz*obj.k*obj.L) /(4*obj.b*obj.k*obj.L));
            r4 = e(3)*obj.Izz/(4*obj.b) + e(2)*obj.Iyy/(2*obj.k*obj.L);
            u = thrust/4 + [r1;r2;r3;r4];
            
            obj.error(:,obj.iteration) = [e_pos;e_angle; thrust];
            
        end
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %SIMULATION FUNCTIONS
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        %demo (not useful) code to show varying position and rotation
        %replace with your own functions!
        function obj = change_pos_and_orientation(obj,followSetpoint)
            %target setpoint
            %mode transition
            switch obj.mode
                case 0
                    %move to mode to 1 if command received, time triggered
                    %now
                    if obj.time >= 1
                       obj.mode = 1;
                    end
                case 1 
                    %if near 0,0,2.5
                 
                    if norm(obj.pos - [0; 0; 2.5]) < 0.1
                        obj.mode = 2;
                    end
                case 2
                    %move to mode to 1 if command received, time triggered
                    %now
                    if obj.time >= 6
                       obj.mode = 3;
                    end
                case 3
                    %if near 0,5,2.5
                    if norm(obj.pos - [5; 0; 2.5]) < 0.1
                        obj.mode = 4;
                    end
                case 4
                    %if near 0,0,2.5
                    if norm(obj.pos - [0; 0; 2.5]) < 0.1
                        obj.mode = 5;
                    end
                    
                case 5
                    %if near 0,0,2.5
                    if norm(obj.pos - [0; 0; 0]) < 0.05
                        %there is no hitbox on the ground, stay at 5
                    end
            end
         
                        
            if obj.mode ~= 6
                %get target setpoint
%                 setpoint = next_setpoint(obj.mode, obj.pos);
                if obj.isFollow
                    setpoint = follow_setpoint(followSetpoint, obj.pos);
                else
                    setpoint = next_setpoint(obj.mode, obj.pos);
                end
                
                %pid controller - with access to all state
                
                obj.u = pid_controller(obj, setpoint);
                
                
            else
                %cutpower to u
                obj.u = [0; 0; 0; 0];
            end
            
            
            %dynamics from input
            obj.omega = Drone.thetadot2omega(obj.thetadot, obj.theta);

            % Compute linear and angular accelerations.
            obj.a = Drone.acceleration(obj.u, obj.theta, obj.xdot, obj.m, obj.g, obj.k, obj.kd);
            obj.omegadot = Drone.angular_acceleration(obj.u, obj.omega, obj.I, obj.L, obj.b, obj.k);

            obj.omega = obj.omega + obj.dt * obj.omegadot;
            obj.thetadot = Drone.omega2thetadot(obj.omega, obj.theta);
            obj.theta = obj.theta + obj.dt * obj.thetadot;
            obj.xdot = obj.xdot + obj.dt * obj.a;
            obj.x = obj.x + obj.dt * obj.xdot;
            obj.state(:,obj.iteration) = [obj.x;obj.xdot;obj.theta;obj.omega];
            
            if(obj.noise_enable)
                noise = randn(3,1)*0.005;
                angledot = obj.thetadot +noise ;
                obj.y = [obj.x;angledot];
                obj.LogNoise(:,obj.iteration) = noise;
            elseif(obj.wind_enable)
                perturbation = obj.Kwind.*(obj.x - obj.wind);
                linear = obj.xdot + perturbation +obj.wc;
                coor = obj.x+obj.dt*linear;
                obj.y = [coor;obj.thetadot];
                obj.LogWind(:,obj.iteration) = [perturbation;obj.wc+perturbation];
                obj.wind = obj.windStrength(obj.wind,obj.Len,obj.dt,obj.Int);
            else
                obj.y = [obj.x;obj.thetadot];   
            end
            obj.integral = obj.integral + obj.dt*obj.y(4:6);
            obj.LogOutput(:,obj.iteration) = obj.y;
            
%             rot_mat = eul2rotm(obj.theta');
            rot_mat = eul2rotm(obj.integral.');
%             obj.pos = obj.x;
            obj.pos = obj.y(1:3);
            obj.R = rot_mat;
        end
        
        
        function update(obj,followSetpoint)
            %update simulation time
            obj.time = obj.time + obj.dt;
            
            %change position and orientation of drone
            obj = change_pos_and_orientation(obj,followSetpoint);
            
            %draw drone on figure
            draw(obj);
            
            obj.iteration = obj.iteration + 1;
        end
        
        
        
        end  
end
