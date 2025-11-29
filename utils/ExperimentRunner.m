classdef ExperimentRunner
    % EXPERIMENTRUNNER Executes Monte Carlo simulations for a given scenario 
    % using one of the four specified filter types.
    
    properties
        dynamics
        sensors
        T           % Time steps
        N_trials    % Monte Carlo runs
        Trajectory
        Controller
    end
    
    methods
        function obj = ExperimentRunner(dynamics, sensors, T, N_trials, pid_params)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.T = T;
            obj.N_trials = N_trials;
            
            % Initialize Trajectory and Controller
            obj.Trajectory = TrajectoryGenerator(dynamics.dt, T);
            obj.Controller = PIDController(pid_params.Kp, pid_params.Ki, pid_params.Kd);
        end
        
        function results = run(obj, filter_name)
            % RUN Executes one Monte Carlo experiment for a specific filter type.
            
            results.errors = [];
            results.P_history = {};
            
            % Determine filter class and if true state is needed for robustness check
            needs_truth = contains(filter_name, 'Robust');
            
            for trial = 1:obj.N_trials
                % Initialize TRUE and ESTIMATE state for each trial
                x_true = [0; 0; 0; 0]; 
                x0 = x_true + mvnrnd(zeros(obj.dynamics.nx,1), diag([1, 1, 0.1, 0.1]))';
                P0 = diag([1.0, 1.0, 0.5, 0.5]);
                
                % Instantiate the appropriate filter
                switch filter_name
                    case 'KF'
                        filter = KalmanFilter(obj.dynamics, obj.sensors, x0, P0);
                    case 'RobustKF'
                        filter = RobustKalmanFilter(obj.dynamics, obj.sensors, x0, P0);
                    case 'EKF'
                        filter = ExtendedKalmanFilter(obj.dynamics, obj.sensors, x0, P0);
                    case 'RobustEKF'
                        filter = RobustEKF(obj.dynamics, obj.sensors, x0, P0);
                    otherwise
                        error('Unknown filter type: %s', filter_name);
                end
                
                trial_errors = zeros(obj.dynamics.nx, obj.T);
                trial_P = cell(1, obj.T);
                u_t_prev = zeros(obj.dynamics.nu, 1);
                
                for t = 1:obj.T
                    x_desired = obj.Trajectory.getDesiredState(t);
                    
                    % 1. Compute Control Input
                    % PID controller uses the current TRUE state to compute control
                    u_t = obj.Controller.computeControl(x_true, x_desired, obj.dynamics.dt);
                    
                    % 2. Filter Prediction
                    filter = filter.predict(u_t_prev); 
                    
                    % 3. Generate measurements and propagate TRUE state
                    w = mvnrnd(zeros(obj.dynamics.nx, 1), obj.dynamics.Q)'; 
                    x_true = obj.dynamics.propagate(x_true, u_t, w);
                    
                    for s = 1:length(obj.sensors)
                        z = obj.sensors{s}.measure(x_true);
                        
                        if needs_truth
                            % Robust filters need x_true for the NEES check
                            filter = filter.update(z, s, x_true); 
                        else
                            filter = filter.update(z, s); 
                        end
                    end
                    
                    % 4. Store results
                    trial_errors(:, t) = x_true - filter.x_hat;
                    trial_P{t} = filter.P;
                    
                    u_t_prev = u_t; 
                end
                
                results.errors = cat(3, results.errors, trial_errors);
                results.P_history = [results.P_history; {trial_P}];
            end
        end
    end
end