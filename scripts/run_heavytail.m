% Main script for heavy-tailed noise experiments
clear; clc; close all;
setup_paths();

fprintf('=== Running Heavy-Tailed Noise Experiments ===\n');

% Test configurations
configs = [
    0.05, 5;   % Rare, moderate outliers
    0.05, 10;  % Rare, severe outliers
    0.1, 10;   % Common, severe outliers
];

for i = 1:size(configs, 1)
    pi_outlier = configs(i, 1);
    lambda = configs(i, 2);
    
    fprintf('\n--- Running pi=%.2f, lambda=%d ---\n', pi_outlier, lambda);
    
    % Get parameters
    params = params_heavytail(pi_outlier, lambda);
    
    % Create dynamics
    dynamics = RobotDynamics(params.dt, params.sigma_w);
    
    % Create sensors with mixture noise
    gps_noise = MixtureNoise(params.sigma_gps^2 * eye(2), pi_outlier, lambda);
    odom_noise = MixtureNoise(params.sigma_odom^2 * eye(2), pi_outlier, lambda);
    
    sensors = {GPSSensor(gps_noise), OdometrySensor(odom_noise)};
    
    % Run experiment
    runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials);
    results = runner.run('standard');
    
    % Save results
    save(sprintf('results/%s.mat', params.scenario), 'results', 'params');
    
    % Generate plots
    plot_scenario_results(results, params, ...
        sprintf('heavytail_pi%.2f_lambda%d', pi_outlier, lambda));
    
    fprintf('Completed pi=%.2f, lambda=%d\n', pi_outlier, lambda);
end

fprintf('\nAll heavy-tailed experiments complete!\n');