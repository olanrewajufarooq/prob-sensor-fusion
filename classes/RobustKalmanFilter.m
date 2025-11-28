classdef RobustKalmanFilter < KalmanFilter
    properties
        inflation_factor    % Current inflation
        nees_buffer        % Recent NEES values
        buffer_size        % Window for adaptive threshold
        delta              % Confidence level
        enable_inflation   % Flag to turn on/off
    end
    
    methods
        function obj = RobustKalmanFilter(dynamics, sensors, x0, P0, delta)
            obj@KalmanFilter(dynamics, sensors, x0, P0);
            if nargin < 5, delta = 0.05; end
            
            obj.delta = delta;
            obj.inflation_factor = 1.0;
            obj.nees_buffer = [];
            obj.buffer_size = 20;
            obj.enable_inflation = true;
        end
        
        function obj = update(obj, z, sensor_idx, x_true)
            % Standard KF update
            obj = update@KalmanFilter(obj, z, sensor_idx);
            
            % Compute NEES if ground truth available
            if nargin > 3 && obj.enable_inflation
                e = x_true - obj.x_hat;
                nees = e' / obj.P * e;
                obj.nees_buffer = [obj.nees_buffer, nees];
                
                % Keep buffer size fixed
                if length(obj.nees_buffer) > obj.buffer_size
                    obj.nees_buffer(1) = [];
                end
                
                % Chebyshev-based threshold
                if length(obj.nees_buffer) >= 10
                    nees_mean = mean(obj.nees_buffer);
                    nees_std = std(obj.nees_buffer);
                    nx = obj.dynamics.nx;
                    
                    % Threshold: mean + k*std where k from Chebyshev
                    k = sqrt(1/obj.delta);
                    threshold = nx + k * nees_std;
                    
                    % Inflate if NEES exceeds threshold
                    if nees > threshold
                        obj.inflation_factor = min(2.0, obj.inflation_factor * 1.1);
                        obj.P = obj.inflation_factor * obj.P;
                    else
                        % Gradually deflate
                        obj.inflation_factor = max(1.0, obj.inflation_factor * 0.99);
                    end
                end
            end
        end
    end
end

