classdef ExtendedKalmanFilter
    
    properties
        x_hat
        P
        dynamics
        sensors
        x_hat_prev 
        epsilon = 1e-9; % Numerical stability constant
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
            obj.x_hat_prev = obj.x_hat; 
            obj.x_hat = obj.dynamics.propagate(obj.x_hat, u_prev, zeros(obj.dynamics.nx, 1));
            Fk = obj.dynamics.getJacobianF(obj.x_hat_prev);
            obj.P = Fk * obj.P * Fk' + obj.dynamics.Q;
            obj.P = obj.P + obj.epsilon * eye(size(obj.P));
        end
        
        function [obj, innovation, S] = update(obj, z, sensor_idx)
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;
            
            if isa(sensor.noise_model, 'CorrelatedGaussianNoise')
                R = sensor.noise_model.getAssumedCovariance();
            else
                R = sensor.noise_model.R;
            end
            
            S = H * obj.P * H' + R;
            K = obj.P * H' / S;
            
            innovation = z - H * obj.x_hat;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            obj.P = 0.5 * (obj.P + obj.P');
        end
    end
end
