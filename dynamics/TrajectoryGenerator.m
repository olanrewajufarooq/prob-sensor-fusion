classdef TrajectoryGenerator
    % TRAJECTORYGENERATOR Defines a desired path (e.g., a circle) 
    % and provides desired states for the PID controller.
    
    properties
        target_path % N_steps x 4 matrix of [px, py, theta, v]_desired
        dt
        type        % Trajectory type string for reference
    end
    
    methods
        function obj = TrajectoryGenerator(dt, T, trajectory_type)
            % T is the number of time steps
            obj.dt = dt;
            obj.type = trajectory_type;
            T_full = T * dt;
            t_vec = (0:dt:T_full)';
            steps = length(t_vec);
            
            % Initialize
            obj.target_path = zeros(steps, 4);

            % Base Parameters
            R = 15; % meters (Base Radius)
            omega = 2*pi / T_full; % Base angular speed (completes one cycle in T_full)

            for k = 1:steps
                t = t_vec(k);
                
                % Initialize derivatives
                vx = 0;
                vy = 0;
                
                switch trajectory_type
                    case 'Figure8'
                        % Figure 8 trajectory (high curvature change)
                        px = R * sin(omega * t);
                        py = R * sin(omega * t) * cos(omega * t); 
                        
                        % Derivatives for velocity/heading calculation
                        vx = R * omega * cos(omega * t);
                        vy = R * omega * (cos(omega * t)^2 - sin(omega * t)^2); 

                    case 'Spiral'
                        % Expanding spiral
                        R_t = R * (0.1 + 0.9 * (t / T_full)); % Radius expands from 1.5m to 15m
                        omega_t = 2 * omega; % Double the base angular speed for a tighter spiral
                        
                        px = R_t * cos(omega_t * t);
                        py = R_t * sin(omega_t * t);
                        
                        % Derivatives (complex due to time-varying radius R_t)
                        R_dot = 0.9 * R / T_full;
                        vx = R_dot * cos(omega_t * t) - R_t * omega_t * sin(omega_t * t);
                        vy = R_dot * sin(omega_t * t) + R_t * omega_t * cos(omega_t * t);
                        
                    case 'Circular'
                        % Simple circular path (low, constant curvature)
                        px = R * cos(omega * t);
                        py = R * sin(omega * t);
                        
                        vx = -R * omega * sin(omega * t);
                        vy = R * omega * cos(omega * t);
                        
                    otherwise
                        error('Invalid trajectory type specified: %s', trajectory_type);
                end
                
                % 2. Calculate desired velocity and heading from derivatives
                v_des = sqrt(vx^2 + vy^2);
                
                % Handle zero velocity case to avoid NaN in atan2 
                if v_des < 1e-6
                    theta_des = obj.target_path(max(1, k-1), 3); % Maintain previous heading
                    v_des = 0;
                else
                    theta_des = atan2(vy, vx);
                end
                
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
