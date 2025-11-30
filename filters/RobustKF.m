classdef RobustKF < KF
    
    properties
        inflation_factor    
        nis_buffer
        buffer_size         
        delta
        outlier_prob_limit = 1e-3; % acute outlier probability limit
        enable_inflation    
    end
    
    methods
        function obj = RobustKF(dynamics, sensors, x0, P0, delta)
            obj@KF(dynamics, sensors, x0, P0);
            if nargin < 5, delta = 0.05; end
            
            obj.delta = delta;
            obj.inflation_factor = 1.0;
            obj.nis_buffer = [];
            obj.buffer_size = 20; 
            obj.enable_inflation = true;
        end
        
        function [obj, innovation, S] = update(obj, z, sensor_idx)
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;

            if isa(sensor.noise_model, 'CorrelatedGaussianNoise')
                R_filt = sensor.noise_model.getAssumedCovariance();
            else
                R_filt = sensor.noise_model.R;
            end

            innovation = z - H * obj.x_hat;
            S_nominal = H * obj.P * H' + R_filt;
            
            expected_sq_norm = trace(S_nominal);
            markov_threshold = expected_sq_norm / obj.outlier_prob_limit;
            
            current_sq_norm = innovation' * innovation;
            if current_sq_norm > markov_threshold
                R_used = R_filt * 100;
                S = H * obj.P * H' + R_used;
            else
                R_used = R_filt;
                S = S_nominal;
            end
            
            K = obj.P * H' / S;
            
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            
            obj.P = 0.5 * (obj.P + obj.P');

            nis = innovation' / S * innovation; 

            if obj.enable_inflation
                obj.nis_buffer = [obj.nis_buffer, nis];
                
                if length(obj.nis_buffer) > obj.buffer_size
                    obj.nis_buffer(1) = [];
                end
                
                if length(obj.nis_buffer) == obj.buffer_size
                    nis_mean = mean(obj.nis_buffer);
                    nz = size(innovation, 1);
                    consistency_threshold = nz * (1 + 2 * sqrt(1/obj.buffer_size)); 
                    
                    if nis_mean > consistency_threshold
                        obj.inflation_factor = min(3.0, obj.inflation_factor * 1.05);
                        obj.P = obj.inflation_factor * obj.P;
                    else
                        obj.inflation_factor = max(1.0, obj.inflation_factor * 0.99);
                    end
                end
            end
        end
    end
end
