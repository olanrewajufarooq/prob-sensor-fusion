classdef KalmanFilter
    % KALMANFILTER Pure Linear KF. Assumes a simplified linear model.
    
    properties
        x_hat
        P
        dynamics    % RobotDynamics object (used for F_linear and Q_linear)
        sensors
        epsilon = 1e-9; % Numerical stability constant
    end
    
    methods
        function obj = KalmanFilter(dynamics, sensors, x0, P0)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.x_hat = x0;
            obj.P = P0;
        end
        
        function obj = predict(obj, u_prev)
            % Time update using the LINEAR F and Q 
            Fk = obj.dynamics.F_linear;
            dt = obj.dynamics.dt;
            
            % Simplified control input assumption for the linear model
            u_linear = [0; 0; u_prev(2)*dt; u_prev(1)*dt]; 
            
            obj.x_hat = Fk * obj.x_hat + u_linear;
            obj.P = Fk * obj.P * Fk' + obj.dynamics.Q_linear;
            
            % Numerical Damping (Regularization)
            obj.P = obj.P + obj.epsilon * eye(size(obj.P));
        end
        
        function [obj, innovation, S] = update(obj, z, sensor_idx)
            % Measurement update
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;
            
            if isa(sensor.noise_model, 'CorrelatedGaussianNoise')
                R = sensor.noise_model.getAssumedCovariance();
            else
                R = sensor.noise_model.R;
            end
            
            % Kalman gain components
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            % Update
            innovation = z - H * obj.x_hat;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            
            % Symmetrization for numerical stability
            obj.P = 0.5 * (obj.P + obj.P');
        end
    end
end