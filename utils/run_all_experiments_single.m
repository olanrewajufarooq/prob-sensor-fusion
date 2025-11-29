function run_all_experiments_single(scenario_label, params_func)
% Helper function to run a single scenario, used by all main scripts.
% Runs KF, RobustKF, EKF, and RobustEKF for a given scenario defined by params_func.

filter_types = {'KF', 'RobustKF', 'EKF', 'RobustEKF'};

params = params_func();
dynamics = RobotDynamics(params.dt, params.Q);

fprintf('\nRunning Scenario: %s (T=%d, N=%d)\n', scenario_label, params.T, params.N_trials);

% --- Determine Subfolders based on scenario type ---
if contains(scenario_label, 'correlated')
    subfolder_name = 'correlated';
    sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, params.rho);
    noise_name = 'Correlated Gaussian';
elseif contains(scenario_label, 'heavytail')
    subfolder_name = 'heavytail';
    R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model = MixtureNoise(R_nominal, params.pi_outlier, params.lambda);
    sensors = {GPSSensor(noise_model)};
    noise_name = 'Mixture/Heavy-Tail';
else 
    subfolder_name = 'baseline'; % Includes 'baseline_gaussian'
    R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model = GaussianNoise(R_nominal);
    sensors = {GPSSensor(noise_model)};
    noise_name = 'Standard Gaussian';
end

plot_subfolder = fullfile('plots', subfolder_name);
results_subfolder = fullfile('results', subfolder_name);

% Ensure output directories exist
if ~exist(plot_subfolder, 'dir')
    mkdir(plot_subfolder);
end
if ~exist(results_subfolder, 'dir')
    mkdir(results_subfolder);
end

% Create Experiment Runner
runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params);

for f = 1:length(filter_types)
    filter_name = filter_types{f};
    
    % Define file path using the subfolder
    results_filename = fullfile(results_subfolder, sprintf('%s_%s.mat', scenario_label, filter_name));
    
    % Check if results already exist to avoid re-running
    if exist(results_filename, 'file')
        fprintf('  Skipping %s (Results already exist).\n', filter_name);
        load(results_filename, 'results');
    else
        fprintf('  Running Filter: %s (Noise: %s)...\n', filter_name, noise_name);
        results = runner.run(filter_name);
        save(results_filename, 'results', 'params', 'filter_name');
    end

    % --- Generate plots for ALL filter types ---
    
     % Recalculate NEES mean across all trials for the plot label
     nees_mean = mean(cell2mat(arrayfun(@(i) Metrics.computeNEES(results.errors(:,:,i), results.P_history{i}), 1:params.N_trials, 'UniformOutput', false)'), 1);
     
     % Get estimated trajectory for plotting (using trial 1)
     x_true_trajectory = runner.Trajectory.getDesiredState(1:params.T);
     x_est_trajectory = x_true_trajectory - results.errors(:,:,1); 
     
     Plotter.plotTrajectory(x_true_trajectory, x_est_trajectory, ...
                            results.P_history{1}, scenario_label, filter_name, plot_subfolder);
     Plotter.plotNEES(nees_mean, dynamics.nx, scenario_label, filter_name, plot_subfolder);
     Plotter.plotErrorHistogram(results.errors, scenario_label, filter_name, plot_subfolder);
end

fprintf('Completed all filters for scenario: %s.\n', scenario_label);

end