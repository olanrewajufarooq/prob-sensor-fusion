classdef TrajectoryGenerator
    % TRAJECTORYGENERATOR Defines a desired path (e.g., a circle) 
    % and provides desired states for the PID controller.
    
    properties
        target_path % N_steps x 4 matrix of [px, py, theta, v]_desired
        dt
    end
    
    methods
        function obj = TrajectoryGenerator(dt, T)
            % T is the number of time steps
            obj.dt = dt;
            T_full = T * dt;
            t_vec = (0:dt:T_full)';
            steps = length(t_vec);
            
            % --- Desired Trajectory: Circular Path ---
            radius = 15; % meters
            angular_speed = 2*pi / T_full; 
            
            obj.target_path = zeros(steps, 4);
            
            for k = 1:steps
                t = t_vec(k);
                
                % 1. Position
                px = radius * cos(angular_speed * t);
                py = radius * sin(angular_speed * t);
                
                % 2. Velocities (for desired state)
                vx = -radius * angular_speed * sin(angular_speed * t);
                vy = radius * angular_speed * cos(angular_speed * t);
                v_des = sqrt(vx^2 + vy^2);

                % 3. Heading (Pointing in the direction of motion)
                theta_des = atan2(vy, vx);
                
                obj.target_path(k, :) = [px, py, theta_des, v_des];
            end
            % Ensure target_path size matches T exactly
            obj.target_path = obj.target_path(1:T, :); 
        end
        
        function x_d = getDesiredState(obj, k)
            % Returns desired state x_d = [px, py, theta, v] at step k
            x_d = obj.target_path(k, :)';
        end
    end
end