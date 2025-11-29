classdef RobustEKF < ExtendedKalmanFilter
    % ROBUST EKF: EKF prediction + Adaptive Covariance Inflation.
    % Uses NIS (Normalized Innovation Squared), a real-time observable, to drive the inflation.
    
    properties
        inflation_factor    
        nis_buffer          % Buffer storing recent NIS values
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
            obj.nis_buffer = [];
            obj.buffer_size = 20; 
            obj.enable_inflation = true;
        end
        
        function obj = predict(obj, u_prev)
            % Uses the EKF prediction
            obj = predict@ExtendedKalmanFilter(obj, u_prev);
        end
        
        function [obj, innovation, S] = update(obj, z, sensor_idx)
            % 1. Standard EKF update (returns innovation and S)
            [obj, innovation, S] = update@ExtendedKalmanFilter(obj, z, sensor_idx);
            
            % --- Robustness Logic (Chebyshev-based Inflation using NIS) ---
            if obj.enable_inflation
                % Calculate NIS: nu_t = r_t' * S^-1 * r_t
                nis = innovation' / S * innovation;
                obj.nis_buffer = [obj.nis_buffer, nis];
                
                % Keep buffer size fixed
                if length(obj.nis_buffer) > obj.buffer_size
                    obj.nis_buffer(1) = [];
                end
                
                % Check consistency if buffer is full
                if length(obj.nis_buffer) == obj.buffer_size
                    nis_mean = mean(obj.nis_buffer);
                    nz = size(innovation, 1); % Measurement dimension
                    
                    % Expected mean NIS is nz. Check against Chebyshev bound.
                    consistency_threshold = nz * (1 + 2 * sqrt(1/obj.buffer_size)); 
                    
                    if nis_mean > consistency_threshold
                        % Inflate P if NIS is consistently too high (inconsistent)
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