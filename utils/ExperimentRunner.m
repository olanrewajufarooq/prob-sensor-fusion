classdef ExperimentRunner
    properties
        dynamics
        sensors
        T           % Time steps
        N_trials    % Monte Carlo runs
    end
    
    methods
        function obj = ExperimentRunner(dynamics, sensors, T, N_trials)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.T = T;
            obj.N_trials = N_trials;
        end
        
        function results = run(obj, filter_type, params)
            if nargin < 3
                params.delta = 0.05; % Default confidence level
            end
            
            results.errors = [];
            results.nees = [];
            results.P_history = {};
            
            for trial = 1:obj.N_trials
                % Initialize
                x_true = [0; 0; 1; 0.5];  % Initial state
                x0 = x_true + mvnrnd(zeros(4,1), eye(4))';
                P0 = eye(4);
                
                if strcmp(filter_type, 'robust')
                    filter = RobustKalmanFilter(obj.dynamics, obj.sensors, x0, P0, params.delta);
                    filter.enable_inflation = true;
                else
                    filter = KalmanFilter(obj.dynamics, obj.sensors, x0, P0);
                end
                
                trial_errors = zeros(4, obj.T);
                trial_P = cell(1, obj.T);
                
                for t = 1:obj.T
                    % Predict
                    filter = filter.predict();
                    
                    % Generate measurements
                    x_true = obj.dynamics.propagate(x_true);
                    
                    for s = 1:length(obj.sensors)
                        z = obj.sensors{s}.measure(x_true);
                        
                        if strcmp(filter_type, 'robust')
                            filter = filter.update(z, s, x_true);
                        else
                            filter = filter.update(z, s);
                        end
                    end
                    
                    % Store results
                    trial_errors(:, t) = x_true - filter.x_hat;
                    trial_P{t} = filter.P;
                end
                
                results.errors = cat(3, results.errors, trial_errors);
                results.P_history = [results.P_history; {trial_P}];
            end
        end
    end
end