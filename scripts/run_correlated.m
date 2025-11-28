% Main script for correlated noise experiments
clear; close all;
setup_paths();

fprintf('=== Running Correlated Noise Experiments ===\n');

% Test multiple correlation values
rho_values = [0.3, 0.5, 0.7, 0.9];

for i = 1:length(rho_values)
    rho = rho_values(i);
    fprintf('\n--- Running rho = %.1f ---\n', rho);
    
    % Get parameters
    params = params_correlated(rho);
    
    % Create dynamics
    dynamics = RobotDynamics(params.dt, params.sigma_w);
    
    % Create correlated noise model
    % Assume GPS and odometry measurements are correlated
    corr_noise = CorrelatedGaussianNoise(params.sigma_gps, params.sigma_odom, rho);
    
    % For simplicity, we'll correlate the position measurements
    % In practice, you might have more complex correlation structure
    gps_noise = GaussianNoise(params.sigma_gps^2 * eye(2));
    odom_noise = GaussianNoise(params.sigma_odom^2 * eye(2));
    
    % Create custom correlated sensor pair
    sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, rho);
    
    % Run experiment
    runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials);
    results = runner.run('standard');
    
    % Save results
    save(sprintf('results/%s.mat', params.scenario), 'results', 'params');
    
    % Generate plots
    plot_scenario_results(results, params, sprintf('correlated_rho%.1f', rho));
    
    fprintf('Completed rho = %.1f\n', rho);
end

fprintf('\nAll correlated experiments complete!\n');