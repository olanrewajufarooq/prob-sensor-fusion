classdef PIDController
    % PIDCONTROLLER Computes the control input u based on the error 
    % between the current state x and the desired state x_d.
    
    properties
        Kp              % Proportional gains [v_c_gain; omega_c_gain]
        Ki              % Integral gains
        Kd              % Derivative gains
        integral_err    
        prev_err        
    end
    
    methods
        function obj = PIDController(Kp, Ki, Kd)
            obj.Kp = Kp; 
            obj.Ki = Ki; 
            obj.Kd = Kd;
            obj.integral_err = zeros(2, 1); 
            obj.prev_err = zeros(2, 1);
        end
        
        function u = computeControl(obj, x_current, x_desired, dt)
            % CONTROL: u = [v_c, omega_c]' (Commanded velocity and turn rate)
            
            % Error in position components
            dpx = x_desired(1) - x_current(1);
            dpy = x_desired(2) - x_current(2);
            
            % 1. Feedback Error:
            % Longitudinal Error (distance to desired point)
            dist_err = sqrt(dpx^2 + dpy^2); 
            % Angular Error (difference between current and desired heading)
            heading_err = wrapToPi(x_desired(3) - x_current(3)); 
            
            err = [dist_err; heading_err]; % Control error vector

            % 2. PID Components
            obj.integral_err = obj.integral_err + err * dt;
            derivative_err = (err - obj.prev_err) / dt;
            
            u_p = obj.Kp .* err;
            u_i = obj.Ki .* obj.integral_err;
            u_d = obj.Kd .* derivative_err;
            
            u = u_p + u_i + u_d;
            
            % 3. Anti-windup and Saturation (Clamping control outputs)
            v_max = 2.0; % 2 m/s
            omega_max = pi/4; % 45 deg/s
            
            u(1) = max(0.0, min(u(1), v_max)); % Velocity is non-negative
            u(2) = max(-omega_max, min(u(2), omega_max));
            
            obj.prev_err = err;
        end
    end
end

function angle = wrapToPi(angle)
    % Helper function to wrap angle to the interval (-pi, pi]
    angle = angle - 2*pi * floor((angle + pi) / (2*pi));
end