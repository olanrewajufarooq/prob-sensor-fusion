% RUN_ROBUST Runs the RobustEKF on a fixed mis-specified scenario while sweeping 
% the Chebyshev confidence parameter (delta).

clear;
close all;

setup_paths();

fprintf('=== Running Robustness Parameter (Delta) Sweep Experiments ===\n');

% Set up Heavy-Tail scenario (pi=0.05, lambda=10)
CORE_SCENARIO_LABEL = 'heavytail_pi0.05_lambda10';
params_func = @(pi, lambda) params_heavytail(0.05, 10);
params = params_func();

% Define the base trajectory type for this sweep
SWEEP_TRAJECTORY = 'Circular'; 

% Delta values (1 - confidence level) for RobustEKF constructor
delta_values = [0.10, 0.05, 0.01]; 
subfolder = 'plots/robust_sweep';

% Define results subfolder and ensure it exists
results_subfolder = 'results/robust_sweep';
if ~exist(results_subfolder, 'dir')
    mkdir(results_subfolder);
end

% --- Setup Sensors for Heavy-Tail Scenario ---
% The RobustEKF needs to be tested on the MixtureNoise model
R_nominal_gps = diag([params.sigma_gps^2, params.sigma_gps^2]);
R_nominal_odom = diag([params.sigma_odom^2, params.sigma_odom^2]);

gps_noise = MixtureNoise(R_nominal_gps, params.pi_outlier, params.lambda);
odom_noise = MixtureNoise(R_nominal_odom, params.pi_outlier, params.lambda);
sensors = {GPSSensor(gps_noise), OdometrySensor(odom_noise)};
% ---------------------------------------------


for i = 1:length(delta_values)
    delta = delta_values(i);
    
    % Create unique scenario label for this delta test
    scenario_label = sprintf('robust_delta_sweep_d%.2f', delta);
    
    fprintf('\n--- Running RobustEKF on %s with Delta = %.2f (Path: %s) ---\n', ...
        CORE_SCENARIO_LABEL, delta, SWEEP_TRAJECTORY);

    % --- Setup System ---
    dynamics = RobotDynamics(params.dt, params.Q);
    
    % Instantiate runner, passing the trajectory type
    runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params, SWEEP_TRAJECTORY);
    
    filter_name = 'RobustEKF';
    results_filename = fullfile(results_subfolder, sprintf('%s_%s.mat', scenario_label, filter_name));
    
    if exist(results_filename, 'file')
        fprintf('  Skipping %s (Results already exist).\n', filter_name);
        load(results_filename, 'results');
    else
        % --- MANUAL RUN LOOP START (Necessary to pass 'delta' to the constructor) ---
        
        % 1. Initialize result storage BEFORE the trial loop
        results.errors = zeros(dynamics.nx, params.T, params.N_trials);
        results.P_history = cell(params.N_trials, 1);
        
        for trial = 1:params.N_trials
            x_true = [0; 0; 0; 0]; 
            x0 = x_true + mvnrnd(zeros(dynamics.nx,1), diag([1, 1, 0.1, 0.1]))';
            P0 = diag([1.0, 1.0, 0.5, 0.5]);
            
            % Instantiate filter with specific delta
            filter = RobustEKF(dynamics, sensors, x0, P0, delta); 
            
            trial_errors = zeros(dynamics.nx, params.T);
            trial_P = cell(1, params.T);
            u_t_prev = zeros(dynamics.nu, 1);
            
            for t = 1:params.T
                x_desired = runner.Trajectory.getDesiredState(t);
                
                % Control uses the current filter estimate (filter.x_hat)
                u_t = runner.Controller.computeControl(filter.x_hat, x_desired, dynamics.dt); 
                
                filter = filter.predict(u_t_prev); 
                
                w = mvnrnd(zeros(dynamics.nx, 1), dynamics.Q)'; 
                x_true = dynamics.propagate(x_true, u_t, w);
                
                for s = 1:length(sensors)
                    z = sensors{s}.measure(x_true);
                    filter = filter.update(z, s); 
                end
                
                trial_errors(:, t) = x_true - filter.x_hat;
                trial_P{t} = filter.P;
                u_t_prev = u_t; 
            end
            
            % Assign results
            results.errors(:, :, trial) = trial_errors;
            results.P_history{trial} = trial_P;
        end
        
        % Save results
        save(results_filename, 'results', 'params', 'filter_name', 'delta');
        fprintf('  Completed and saved results for Delta = %.2f.\n', delta);
        
        % --- MANUAL RUN LOOP END ---
    end
    
    % --- PLOTTING SECTION ---
    
    % 1. Calculate NEES mean across all trials for the plot label (analysis only)
    nees_all_trials = cell2mat(arrayfun(@(i) Metrics.computeNEES(results.errors(:,:,i), results.P_history{i}), 1:params.N_trials, 'UniformOutput', false)');
    nees_mean = mean(nees_all_trials, 1);
    
    % 2. Get trajectories for plotting (using trial 1)
    x_true_trajectory = runner.Trajectory.getDesiredState(1:params.T);
    x_est_trajectory = x_true_trajectory - results.errors(:,:,1); 
    P_history_trial1 = results.P_history{1};
    
    Plotter.plotTrajectory(x_true_trajectory, x_est_trajectory, ...
                           P_history_trial1, scenario_label, filter_name, subfolder);
                           
    Plotter.plotNEES(nees_mean, dynamics.nx, scenario_label, filter_name, subfolder);
    Plotter.plotErrorHistogram(results.errors, scenario_label, filter_name, subfolder);

end

fprintf('\nAll robustness parameter sweep experiments complete!\n');