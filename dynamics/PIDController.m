classdef PIDController
    
    properties
        Kp
        Ki
        Kd
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
            dpx = x_desired(1) - x_current(1);
            dpy = x_desired(2) - x_current(2);
            dist_err = sqrt(dpx^2 + dpy^2); 
            heading_err = wrapToPi(x_desired(3) - x_current(3)); 
            
            err = [dist_err; heading_err];

            obj.integral_err = obj.integral_err + err * dt;
            derivative_err = (err - obj.prev_err) / dt;
            
            u_p = obj.Kp .* err;
            u_i = obj.Ki .* obj.integral_err;
            u_d = obj.Kd .* derivative_err;
            
            u = u_p + u_i + u_d;
            
            v_max = 2.0;
            omega_max = pi/4;
            
            u(1) = max(0.0, min(u(1), v_max));
            u(2) = max(-omega_max, min(u(2), omega_max));
            
            obj.prev_err = err;
        end
    end
end

function angle = wrapToPi(angle)
    angle = angle - 2*pi * floor((angle + pi) / (2*pi));
end
