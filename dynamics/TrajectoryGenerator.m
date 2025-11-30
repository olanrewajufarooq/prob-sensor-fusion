classdef TrajectoryGenerator
    
    properties
        target_path
        dt
        type
    end
    
    methods
        function obj = TrajectoryGenerator(dt, T, trajectory_type)
            obj.dt = dt;
            obj.type = trajectory_type;
            T_full = T * dt;
            t_vec = (0:dt:T_full)';
            steps = length(t_vec);
            
            obj.target_path = zeros(steps, 4);

            R = 15;
            omega = 2*pi / T_full;

            for k = 1:steps
                t = t_vec(k);
                vx = 0;
                vy = 0;
                
                switch trajectory_type
                    case 'Figure8'
                        px = R * sin(omega * t);
                        py = R * sin(omega * t) * cos(omega * t); 
                        vx = R * omega * cos(omega * t);
                        vy = R * omega * (cos(omega * t)^2 - sin(omega * t)^2); 

                    case 'Spiral'
                        R_t = R * (0.1 + 0.9 * (t / T_full));
                        omega_t = 2 * omega;
                        px = R_t * cos(omega_t * t);
                        py = R_t * sin(omega_t * t);
                        R_dot = 0.9 * R / T_full;
                        vx = R_dot * cos(omega_t * t) - R_t * omega_t * sin(omega_t * t);
                        vy = R_dot * sin(omega_t * t) + R_t * omega_t * cos(omega_t * t);
                        
                    case 'Circular'
                        px = R * cos(omega * t);
                        py = R * sin(omega * t);
                        vx = -R * omega * sin(omega * t);
                        vy = R * omega * cos(omega * t);
                        
                    otherwise
                        error('Invalid trajectory type specified: %s', trajectory_type);
                end
                
                v_des = sqrt(vx^2 + vy^2);
                if v_des < 1e-6
                    theta_des = obj.target_path(max(1, k-1), 3);
                    v_des = 0;
                else
                    theta_des = atan2(vy, vx);
                end
                
                obj.target_path(k, :) = [px, py, theta_des, v_des];
            end
            
            obj.target_path = obj.target_path(1:T, :); 
        end
        
        function x_d = getDesiredState(obj, k)
            x_d = obj.target_path(k, :)';
        end
    end
end
