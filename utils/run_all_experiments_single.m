function run_all_experiments_single(scenario_label, params_func)
% Helper function to run a single scenario, used by all main scripts.
% Runs KF, RobustKF, EKF, and RobustEKF for a given scenario defined by params_func.

filter_types = {'KF', 'RobustKF', 'EKF', 'RobustEKF'};
plot_folder = 'plots';

params = params_func();
dynamics = RobotDynamics(params.dt, params.Q);

% --- Setup Sensor/Noise Model based on scenario ---
if contains(scenario_label, 'correlated')
    sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, params.rho);
    noise_name = 'Correlated Gaussian';
elseif contains(scenario_label, 'heavytail')
    R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model = MixtureNoise(R_nominal, params.pi_outlier, params.lambda);
    sensors = {GPSSensor(noise_model)};
    noise_name = 'Mixture/Heavy-Tail';
else 
    R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model = GaussianNoise(R_nominal);
    sensors = {GPSSensor(noise_model)};
    noise_name = 'Standard Gaussian';
end

% Create Experiment Runner
runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params);

for f = 1:length(filter_types)
    filter_name = filter_types{f};
    
    % Check if results already exist to avoid re-running
    results_filename = sprintf('results/%s_%s.mat', scenario_label, filter_name);
    if exist(results_filename, 'file')
        fprintf('  Skipping %s (Results already exist).\n', filter_name);
        load(results_filename, 'results');
    else
        fprintf('  Running Filter: %s (Noise: %s)...\n', filter_name, noise_name);
        results = runner.run(filter_name);
        save(results_filename, 'results', 'params', 'filter_name');
    end

    % --- Generate plots for ALL filter types ---
    % Plotting is now performed for KF, RobustKF, EKF, and RobustEKF 
    % to visualize the effects of both linearity and noise mis-specification.
    
    nees_mean = mean(cell2mat(arrayfun(@(i) Metrics.computeNEES(results.errors(:,:,i), results.P_history{i}), 1:params.N_trials, 'UniformOutput', false)'), 1);
    
    % The estimated state x_est = x_true - error. We use the desired state (proxy for true state)
    x_true_trajectory = runner.Trajectory.getDesiredState(1:params.T);
    x_est_trajectory = x_true_trajectory - results.errors(:,:,1); 
    
    Plotter.plotTrajectory(x_true_trajectory, x_est_trajectory, ...
                        results.P_history{1}, scenario_label, filter_name, plot_folder);
    Plotter.plotNEES(nees_mean, dynamics.nx, scenario_label, filter_name, plot_folder);
    Plotter.plotErrorHistogram(results.errors, scenario_label, filter_name, plot_folder);
end

fprintf('Completed all filters for scenario: %s.\n', scenario_label);

end