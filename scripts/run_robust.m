% RUN_ROBUST Runs the RobustEKF on a fixed mis-specified scenario while sweeping 
% the Chebyshev confidence parameter (delta).
% This demonstrates the trade-off between robustness (inflation) and nominal performance.

clc;
clear all;
close all;

setup_paths();

fprintf('=== Running Robustness Parameter (Delta) Sweep Experiments ===\n');

% Fixed mis-specified scenario to test robustness adaptation
CORE_SCENARIO_LABEL = 'correlated_rho0.7';
params_func = @(r) params_correlated(0.7);
params = params_func();

% Delta values (1 - confidence level) for RobustEKF constructor
delta_values = [0.10, 0.05, 0.01]; 

for i = 1:length(delta_values)
    delta = delta_values(i);
    
    % Create unique scenario label for this delta test
    scenario_label = sprintf('robust_delta_sweep_d%.2f', delta);
    
    fprintf('\n--- Running RobustEKF on %s with Delta = %.2f ---\n', CORE_SCENARIO_LABEL, delta);

    % --- Setup System ---
    dynamics = RobotDynamics(params.dt, params.Q);
    sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, params.rho);
    runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params);
    
    filter_name = 'RobustEKF';
    plot_folder = 'plots';
    results_filename = sprintf('results/%s_%s.mat', scenario_label, filter_name);
    
    if exist(results_filename, 'file')
        fprintf('  Skipping %s (Results already exist).\n', filter_name);
        load(results_filename, 'results');
    else
        % --- MANUAL RUN LOOP START (to pass delta) ---
        
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
                u_t = runner.Controller.computeControl(x_true, x_desired, dynamics.dt);
                
                filter = filter.predict(u_t_prev); 
                
                w = mvnrnd(zeros(dynamics.nx, 1), dynamics.Q)'; 
                x_true = dynamics.propagate(x_true, u_t, w);
                
                for s = 1:length(sensors)
                    z = sensors{s}.measure(x_true);
                    filter = filter.update(z, s, x_true); % Robust update needs x_true
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
    
    % 1. Calculate NEES mean across all trials for the plot label
    nees_all_trials = cell2mat(arrayfun(@(i) Metrics.computeNEES(results.errors(:,:,i), results.P_history{i}), 1:params.N_trials, 'UniformOutput', false)');
    nees_mean = mean(nees_all_trials, 1);
    
    % 2. Get trajectories for plotting (using trial 1)
    x_true_trajectory = runner.Trajectory.getDesiredState(1:params.T);
    x_est_trajectory = x_true_trajectory - results.errors(:,:,1); 
    P_history_trial1 = results.P_history{1};
    
    Plotter.plotTrajectory(x_true_trajectory, x_est_trajectory, ...
                           P_history_trial1, scenario_label, filter_name, plot_folder);
                           
    Plotter.plotNEES(nees_mean, dynamics.nx, scenario_label, filter_name, plot_folder);
    Plotter.plotErrorHistogram(results.errors, scenario_label, filter_name, plot_folder);

end

fprintf('\nAll robustness parameter sweep experiments complete!\n');