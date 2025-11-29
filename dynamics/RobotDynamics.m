classdef RobotDynamics
    % ROBOTDYNAMICS Non-linear discrete-time dynamics for a 2D unicycle robot.
    % TRUE STATE MODEL: Non-linear propagation
    % LINEAR FILTER MODEL: Linear CV propagation (used by KF/RobustKF for comparison)
    
    properties
        Q           % Motion model noise covariance (used by EKF/REKF)
        dt          % Time step
        nx          % State dimension
        nu          % Control dimension
        
        F_linear    % Linear Constant Velocity F matrix (for KF/RKF comparison)
        Q_linear    % Linear Q matrix (for KF/RKF comparison)
    end
    
    methods
        function obj = RobotDynamics(dt, model_noise_Q)
            % model_noise_Q is the true Q used in simulation and the assumed Q for EKF/REKF
            obj.dt = dt;
            obj.nx = 4;
            obj.nu = 2;
            obj.Q = model_noise_Q;
            
            % --- Linear Model F and Q for KF/RobustKF ---
            % KF/RobustKF assume a simplified linear Constant Velocity model for prediction
            obj.F_linear = [
                1, 0, 0, dt;
                0, 1, 0, 0;
                0, 0, 1, 0;
                0, 0, 0, 1;
            ]; 
            obj.Q_linear = model_noise_Q; 
        end
        
        function x_next = propagate(obj, x, u, w)
            % PROPAGATE Non-linear state update (Unicycle model): x_next = f(x, u) + w
            
            dt = obj.dt;
            theta = x(3);
            v_c = u(1);     % Commanded velocity
            omega_c = u(2); % Commanded turn rate
            
            % Non-linear part f(x, u)
            f_x_u = x + [
                x(4) * cos(theta) * dt; % px
                x(4) * sin(theta) * dt; % py
                omega_c * dt;           % theta
                v_c * dt                % v
            ];
            
            x_next = f_x_u + w; % Additive process noise
        end
        
        function Fk = getJacobianF(obj, x_prev)
            % Jacobian F_k = df/dx |_(x_prev) for EKF
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