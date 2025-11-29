classdef RobustEKF < ExtendedKalmanFilter
    % ROBUST EKF: EKF prediction + Adaptive Covariance Inflation.
    % Uses EKF for prediction, adds Chebyshev-based inflation to P.
    
    properties
        inflation_factor    
        nees_buffer         
        buffer_size         
        delta               
        enable_inflation    
    end
    
    methods
        function obj = RobustEKF(dynamics, sensors, x0, P0, delta)
            obj@ExtendedKalmanFilter(dynamics, sensors, x0, P0);
            if nargin < 5, delta = 0.05; end
            
            obj.delta = delta;
            obj.inflation_factor = 1.0;
            obj.nees_buffer = [];
            obj.buffer_size = 20; 
            obj.enable_inflation = true;
        end
        
        function obj = predict(obj, u_prev)
            % Uses the EKF prediction
            obj = predict@ExtendedKalmanFilter(obj, u_prev);
        end
        
        function [obj, innovation] = update(obj, z, sensor_idx, x_true)
            % Standard EKF update
            [obj, innovation] = update@ExtendedKalmanFilter(obj, z, sensor_idx);
            
            % --- Robustness Logic (Chebyshev-based Inflation) ---
            if nargin > 3 && obj.enable_inflation
                e = x_true - obj.x_hat;
                nees = e' / obj.P * e;
                obj.nees_buffer = [obj.nees_buffer, nees];
                
                if length(obj.nees_buffer) > obj.buffer_size
                    obj.nees_buffer(1) = [];
                end
                
                if length(obj.nees_buffer) == obj.buffer_size
                    nees_mean = mean(obj.nees_buffer);
                    nx = obj.dynamics.nx; 
                    
                    % Chebyshev Threshold check (Factor of Safety derived from concentration)
                    consistency_threshold = nx * (1 + 2 * sqrt(1/obj.buffer_size)); 
                    
                    if nees_mean > consistency_threshold
                        % Inflate P
                        obj.inflation_factor = min(3.0, obj.inflation_factor * 1.05);
                        obj.P = obj.inflation_factor * obj.P;
                    else
                        % Gradually deflate P if consistent
                        obj.inflation_factor = max(1.0, obj.inflation_factor * 0.99);
                    end
                end
            end
        end
    end
end