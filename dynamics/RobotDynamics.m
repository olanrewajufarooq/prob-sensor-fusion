classdef RobotDynamics
    
    properties
        Q
        dt
        nx
        nu
        F_linear
        Q_linear
    end
    
    methods
        function obj = RobotDynamics(dt, model_noise_Q)
            obj.dt = dt;
            obj.nx = 4;
            obj.nu = 2;
            obj.Q = model_noise_Q;
            obj.F_linear = [
                1, 0, 0, dt;
                0, 1, 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1;
            ]; 
            obj.Q_linear = model_noise_Q; 
        end
        
        function x_next = propagate(obj, x, u, w)
            dt = obj.dt;
            theta = x(3);
            v_c = u(1);
            omega_c = u(2);
            f_x_u = x + [
                x(4) * cos(theta) * dt;
                x(4) * sin(theta) * dt;
                omega_c * dt;
                v_c * dt
            ];
            x_next = f_x_u + w;
        end
        
        function Fk = getJacobianF(obj, x_prev)
            dt = obj.dt;
            theta = x_prev(3);
            v = x_prev(4);

            Fk = [
                1, 0, -v*sin(theta)*dt, cos(theta)*dt;
                0, 1, v*cos(theta)*dt, sin(theta)*dt;
                0, 0, 1, 0;
                0, 0, 0, 1
            ];
        end
    end
end
