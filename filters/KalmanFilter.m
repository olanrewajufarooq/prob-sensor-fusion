classdef KalmanFilter
    properties
        x_hat       % State estimate
        P           % Covariance estimate
        dynamics    % RobotDynamics object
        sensors     % Cell array of SensorModel objects
    end
    
    methods
        function obj = KalmanFilter(dynamics, sensors, x0, P0)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.x_hat = x0;
            obj.P = P0;
        end
        
        function obj = predict(obj)
            obj.x_hat = obj.dynamics.F * obj.x_hat;
            obj.P = obj.dynamics.F * obj.P * obj.dynamics.F' + obj.dynamics.Q;
        end
        
        function obj = update(obj, z, sensor_idx)
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;
            
            % Use assumed covariance for filter
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
