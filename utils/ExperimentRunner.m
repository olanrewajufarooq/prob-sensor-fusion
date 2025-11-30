classdef ExperimentRunner
    
    properties
        dynamics
        sensors
        T
        N_trials
        Trajectory
        Controller
        robust_config      % Configuration for robust filters
    end
    
    methods
        function obj = ExperimentRunner(dynamics, sensors, T, N_trials, pid_params, trajectory_type, varargin)
            obj.dynamics = dynamics;
            obj.sensors = sensors;
            obj.T = T;
            obj.N_trials = N_trials;
            
            obj.Trajectory = TrajectoryGenerator(dynamics.dt, T, trajectory_type);
            obj.Controller = PIDController(pid_params.Kp, pid_params.Ki, pid_params.Kd);
            
            % Default robust configuration
            obj.robust_config = struct();
            obj.robust_config.delta = 1e-3;
            obj.robust_config.buffer_size = 20;
            obj.robust_config.inflation_cap = 3.0;
            obj.robust_config.inflation_rate = 1.05;
            obj.robust_config.deflation_rate = 0.99;
            obj.robust_config.markov_inflation = 100;
            
            % Parse optional configuration
            if ~isempty(varargin)
                config = varargin{1};
                fields = fieldnames(config);
                for i = 1:length(fields)
                    obj.robust_config.(fields{i}) = config.(fields{i});
                end
            end
        end
        
        function results = run(obj, filter_name)
            results.errors = zeros(obj.dynamics.nx, obj.T, obj.N_trials);
            results.P_history = cell(obj.N_trials, 1);
            results.traj_true = cell(obj.N_trials, 1);
            results.traj_hat = cell(obj.N_trials, 1);
            results.u_history = cell(obj.N_trials, 1);
            results.dt = obj.dynamics.dt;
            results.trajectory_type = obj.Trajectory.type;
            
            for trial = 1:obj.N_trials
                x_true = [0; 0; 0; 0]; 
                x0 = x_true + mvnrnd(zeros(obj.dynamics.nx,1), diag([1, 1, 0.1, 0.1]))';
                P0 = diag([1.0, 1.0, 0.5, 0.5]);
                
                switch filter_name
                    case 'KF'
                        filter = KF(obj.dynamics, obj.sensors, x0, P0);
                    case 'RobustKF'
                        filter = RobustKF(obj.dynamics, obj.sensors, x0, P0, ...
                            'delta', obj.robust_config.delta, ...
                            'buffer_size', obj.robust_config.buffer_size, ...
                            'inflation_cap', obj.robust_config.inflation_cap, ...
                            'inflation_rate', obj.robust_config.inflation_rate, ...
                            'deflation_rate', obj.robust_config.deflation_rate, ...
                            'markov_inflation', obj.robust_config.markov_inflation);
                    case 'EKF'
                        filter = ExtendedKalmanFilter(obj.dynamics, obj.sensors, x0, P0);
                    case 'RobustEKF'
                        filter = RobustEKF(obj.dynamics, obj.sensors, x0, P0, ...
                            'delta', obj.robust_config.delta, ...
                            'buffer_size', obj.robust_config.buffer_size, ...
                            'inflation_cap', obj.robust_config.inflation_cap, ...
                            'inflation_rate', obj.robust_config.inflation_rate, ...
                            'deflation_rate', obj.robust_config.deflation_rate, ...
                            'markov_inflation', obj.robust_config.markov_inflation);
                    otherwise
                        error('Unknown filter type: %s', filter_name);
                end
                
                trial_errors = zeros(obj.dynamics.nx, obj.T);
                trial_P = cell(1, obj.T);
                trial_x_true = zeros(obj.dynamics.nx, obj.T);
                trial_x_hat = zeros(obj.dynamics.nx, obj.T);
                trial_u = zeros(obj.dynamics.nu, obj.T);
                u_t_prev = zeros(obj.dynamics.nu, 1);
                
                for t = 1:obj.T
                    x_desired = obj.Trajectory.getDesiredState(t);
                    
                    u_t = obj.Controller.computeControl(filter.x_hat, x_desired, obj.dynamics.dt); 
                    
                    filter = filter.predict(u_t_prev); 
                    
                    w = mvnrnd(zeros(obj.dynamics.nx, 1), obj.dynamics.Q)'; 
                    x_true = obj.dynamics.propagate(x_true, u_t, w);
                    
                    for s = 1:length(obj.sensors)
                        z = obj.sensors{s}.measure(x_true);
                        
                        filter = filter.update(z, s); 
                    end
                    
                    trial_errors(:, t) = x_true - filter.x_hat;
                    trial_P{t} = filter.P;
                    trial_x_true(:, t) = x_true;
                    trial_x_hat(:, t) = filter.x_hat;
                    trial_u(:, t) = u_t;
                    
                    u_t_prev = u_t;
                end
                
                results.errors(:, :, trial) = trial_errors;
                results.P_history{trial} = trial_P;
                results.traj_true{trial} = trial_x_true;
                results.traj_hat{trial} = trial_x_hat;
                results.u_history{trial} = trial_u;
            end
        end
    end
end
