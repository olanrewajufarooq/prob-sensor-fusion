classdef RobustEKF < ExtendedKalmanFilter
    
    properties
        % Markov-based acute outlier rejection
        delta                      % Outlier probability bound
        markov_inflation_factor    % How much to inflate R when outlier detected
        
        % Chebyshev-based chronic inconsistency detection
        inflation_factor           % Current state covariance inflation factor
        nis_buffer                 % Sliding window of NIS values
        buffer_size                % Chebyshev window size
        inflation_cap              % Maximum allowed inflation factor
        inflation_rate             % Rate of inflation increase (per trigger)
        deflation_rate             % Rate of deflation decay
        
        enable_inflation           % Flag to enable adaptive covariance inflation
    end
    
    methods
        function obj = RobustEKF(dynamics, sensors, x0, P0, varargin)
            % Constructor with optional tuning parameters
            % Usage: RobustEKF(dynamics, sensors, x0, P0, 'delta', 0.01, 'buffer_size', 30, ...)
            
            obj@ExtendedKalmanFilter(dynamics, sensors, x0, P0);
            
            % Default robustness parameters
            obj.delta = 1e-3;              % Markov outlier threshold (lower = more aggressive)
            obj.markov_inflation_factor = 100;
            obj.buffer_size = 20;          % Chebyshev window size
            obj.inflation_cap = 3.0;       % Max inflation factor
            obj.inflation_rate = 1.05;     % 5% increase per trigger
            obj.deflation_rate = 0.99;     % 1% decrease per normal step
            
            obj.inflation_factor = 1.0;
            obj.nis_buffer = [];
            obj.enable_inflation = true;
            
            % Parse optional arguments for tuning
            p = inputParser();
            addParameter(p, 'delta', obj.delta);
            addParameter(p, 'buffer_size', obj.buffer_size);
            addParameter(p, 'inflation_cap', obj.inflation_cap);
            addParameter(p, 'inflation_rate', obj.inflation_rate);
            addParameter(p, 'deflation_rate', obj.deflation_rate);
            addParameter(p, 'markov_inflation', obj.markov_inflation_factor);
            
            parse(p, varargin{:});
            
            obj.delta = p.Results.delta;
            obj.buffer_size = p.Results.buffer_size;
            obj.inflation_cap = p.Results.inflation_cap;
            obj.inflation_rate = p.Results.inflation_rate;
            obj.deflation_rate = p.Results.deflation_rate;
            obj.markov_inflation_factor = p.Results.markov_inflation;
        end
        
        function obj = predict(obj, u_prev)
            obj = predict@ExtendedKalmanFilter(obj, u_prev);
        end
        
        function [obj, innovation, S] = update(obj, z, sensor_idx)
            sensor = obj.sensors{sensor_idx};
            H = sensor.H;

            % Get assumed measurement noise covariance
            if isa(sensor.noise_model, 'CorrelatedGaussianNoise')
                R_filt = sensor.noise_model.getAssumedCovariance();
            else
                R_filt = sensor.noise_model.R;
            end

            % ===== DEFENSE 2: Chronic Inconsistency - Apply inflation BEFORE update =====
            % When chronic inconsistency is detected, inflate R to reduce filter gain
            % This maintains filter stability by properly adjusting the innovation covariance
            R_chronic = R_filt * obj.inflation_factor;
            
            % Compute innovation using prior (predicted) state
            innovation = z - H * obj.x_hat;
            S_nominal = H * obj.P * H' + R_chronic;
            
            % ===== DEFENSE 1: Acute Outlier Rejection (Markov-based) =====
            expected_sq_norm = trace(S_nominal);
            markov_threshold = expected_sq_norm / obj.delta;
            
            current_sq_norm = innovation' * innovation;
            if current_sq_norm > markov_threshold
                % Outlier detected: inflate measurement noise to downweight this measurement
                R_used = R_chronic * obj.markov_inflation_factor;  %#ok<NASGU>
                S = H * obj.P * H' + R_used;
            else
                R_used = R_chronic;  %#ok<NASGU> % Nominal case with chronic inflation applied
                S = S_nominal;
            end
            
            % Standard Kalman update with properly adjusted R
            K = obj.P * H' / S;
            obj.x_hat = obj.x_hat + K * innovation;
            obj.P = (eye(size(obj.P)) - K * H) * obj.P;
            obj.P = 0.5 * (obj.P + obj.P');  % Enforce symmetry
            
            % ===== Update chronic inconsistency detection state =====
            nis = innovation' / S * innovation;

            if obj.enable_inflation
                % Maintain sliding window of NIS values
                obj.nis_buffer = [obj.nis_buffer, nis];
                
                if length(obj.nis_buffer) > obj.buffer_size
                    obj.nis_buffer(1) = [];
                end
                
                % Check consistency when window is full and update inflation factor for NEXT step
                if length(obj.nis_buffer) == obj.buffer_size
                    nis_mean = mean(obj.nis_buffer);
                    nz = size(innovation, 1);
                    
                    % Chebyshev threshold for detecting chronic inconsistency
                    consistency_threshold = nz * (1 + 2 * sqrt(1/obj.buffer_size));
                    
                    if nis_mean > consistency_threshold
                        % Chronic underestimation: increase inflation factor for next step
                        obj.inflation_factor = min(obj.inflation_cap, obj.inflation_factor * obj.inflation_rate);
                    else
                        % Consistent: allow gradual deflation
                        obj.inflation_factor = max(1.0, obj.inflation_factor * obj.deflation_rate);
                    end
                end
            end
        end
    end
end

