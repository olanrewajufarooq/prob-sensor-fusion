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
        function obj = ExperimentRunner(dynamics, sensors, T, N_trials, pid_params, trajectory_type)
            % Constructor: Initializes the simulation environment and controller.
            % Accepts trajectory_type string to define the path (e.g., 'Circular').
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.T = T;
            obj.N_trials = N_trials;
            
            % 1. Initialize Trajectory, passing the path type string
            obj.Trajectory = TrajectoryGenerator(dynamics.dt, T, trajectory_type);
            
            % 2. Initialize Controller
            obj.Controller = PIDController(pid_params.Kp, pid_params.Ki, pid_params.Kd);
        end
        
        function results = run(obj, filter_name)
            % RUN Executes one Monte Carlo experiment for a specific filter type.
            
            % Pre-allocate storage for efficiency
            results.errors = zeros(obj.dynamics.nx, obj.T, obj.N_trials);
            results.P_history = cell(obj.N_trials, 1);
            
            for trial = 1:obj.N_trials
                % Initialize TRUE and ESTIMATE state for each trial
                x_true = [0; 0; 0; 0]; 
                x0 = x_true + mvnrnd(zeros(obj.dynamics.nx,1), diag([1, 1, 0.1, 0.1]))';
                P0 = diag([1.0, 1.0, 0.5, 0.5]);
                
                % Instantiate the appropriate filter based on filter_name
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
                    
                    % 1. Compute Control Input: Uses the current filter estimate (filter.x_hat) 
                    % for real-world closed-loop simulation.
                    u_t = obj.Controller.computeControl(filter.x_hat, x_desired, obj.dynamics.dt); 
                    
                    % 2. Filter Prediction
                    filter = filter.predict(u_t_prev); 
                    
                    % 3. Generate measurements and propagate TRUE state
                    w = mvnrnd(zeros(obj.dynamics.nx, 1), obj.dynamics.Q)'; 
                    x_true = obj.dynamics.propagate(x_true, u_t, w);
                    
                    for s = 1:length(obj.sensors)
                        z = obj.sensors{s}.measure(x_true);
                        
                        % Filter Update: Uses only observable measurements (z, s).
                        % Robustness check (NIS) is handled internally.
                        filter = filter.update(z, s); 
                    end
                    
                    % 4. Store metrics (True error is calculated externally for analysis)
                    trial_errors(:, t) = x_true - filter.x_hat;
                    trial_P{t} = filter.P;
                    
                    u_t_prev = u_t; 
                end
                
                % Assign results using subscript indexing
                results.errors(:, :, trial) = trial_errors;
                results.P_history{trial} = trial_P;
            end
        end
    end
end