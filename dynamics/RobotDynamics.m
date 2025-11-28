classdef RobotDynamics
    %ROBOTDYNAMICS Linear discrete-time dynamics for 2d robot
    %   State: x = [px, py, vx, vy]'
    %   Model: \dot{x} = F.x + Q . w
    
    properties
        F   % State Transition Matrix
        Q   % Motion model noise covariance
        dt  % Time step
        nx  % State dimension
    end
    
    methods
        function obj = RobotDynamics(dt, model_noise_std)
            %ROBOTDYNAMICS Construct an instance of this class
            %   dt: intervals
            %   model_noise_std: standard deviation of motion model
            obj.dt = dt;
            obj.nx = 4;

            % Constant Velocity Model
            obj.F = [
                1, 0, dt, 0;
                0, 1, 0, dt;
                0, 0, 1, 0;
                0, 0, 0, 1;
            ];
            
            % Motion Model Noise
            q = model_noise_std^2;
            obj.Q = q * [
                dt^3/3, 0, dt^2/2, 0;
                0, dt^3/3, 0, dt^2/2;
                dt^2/2, 0, dt, 0;
                0, dt^2/2, 0, dt
            ];
        end
        
        function x_next = propagate(obj, x, w)
            %PROPAGATE Propagate state with an optional motion model noise
            %   x:          state
            %   w:          motion model noise
            %   x_next:     OUTPUT

            if nargin < 3
                w = mvnrnd(zeros(obj.nx, 1), obj.Q)';
            end

            x_next = obj.F * x + w;
        end
    end
end

