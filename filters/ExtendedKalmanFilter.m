classdef ExtendedKalmanFilter
    % EXTENDEDKALMANFILTER Standard EKF for non-linear dynamics.
    % Assumes Gaussian noise and linear measurements.
    
    properties
        x_hat
        P
        dynamics
        sensors
        x_hat_prev 
    end
    
    methods
        function obj = ExtendedKalmanFilter(dynamics, sensors, x0, P0)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.x_hat = x0;
            obj.P = P0;
            obj.x_hat_prev = x0;
        end
        
        function obj = predict(obj, u_prev)
            % Time update using non-linear function f and Jacobian Fk
            
            obj.x_hat_prev = obj.x_hat; 
            
            % 1. Non-linear state prediction (w=0 for prediction)
            % Uses the true non-linear model from RobotDynamics
            obj.x_hat = obj.dynamics.propagate(obj.x_hat, u_prev, zeros(obj.dynamics.nx, 1));
            
            % 2. Covariance prediction using Jacobian F_k
            Fk = obj.dynamics.getJacobianF(obj.x_hat_prev);
            obj.P = Fk * obj.P * Fk' + obj.dynamics.Q; 
        end
        
        function [obj, innovation] = update(obj, z, sensor_idx)
            % Measurement update (H is linear)
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;
            
            % R logic: uses assumed covariance
            if isa(sensor.noise_model, 'CorrelatedGaussianNoise')
                R = sensor.noise_model.getAssumedCovariance();
            else
                R = sensor.noise_model.R;
            end
            
            % Kalman gain
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            % Update
            innovation = z - H * obj.x_hat;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
        end
    end
end