classdef KF
    
    properties
        x_hat
        P
        dynamics
        sensors
        epsilon = 1e-9; % numerical stabilizer
    end
    
    methods
        function obj = KF(dynamics, sensors, x0, P0)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.x_hat = x0;
            obj.P = P0;
        end
        
        function obj = predict(obj, u_prev)
            Fk = obj.dynamics.F_linear;
            dt = obj.dynamics.dt;
            
            u_linear = [0; 0; u_prev(2)*dt; u_prev(1)*dt]; 
            
            obj.x_hat = Fk * obj.x_hat + u_linear;
            obj.P = Fk * obj.P * Fk' + obj.dynamics.Q_linear;
            
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
