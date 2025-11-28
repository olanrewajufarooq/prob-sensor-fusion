% Main script for robust filter experiments
clear; clc; close all;
setup_paths();

fprintf('=== Running Robust Filter Experiments ===\n');

% Test on correlated noise (rho=0.7) and heavy-tailed (pi=0.05, lambda=10)
test_cases = {
    struct('type', 'correlated', 'rho', 0.7), ...
    struct('type', 'heavytail', 'pi', 0.05, 'lambda', 10)
};

for i = 1:length(test_cases)
    test_case = test_cases{i};
    
    fprintf('\n--- Running robust filter on %s ---\n', test_case.type);
    
    if strcmp(test_case.type, 'correlated')
        % Correlated noise case
        params = params_correlated(test_case.rho);
        params.scenario = sprintf('robust_%s_rho%.1f', test_case.type, test_case.rho);
        
        dynamics = RobotDynamics(params.dt, params.sigma_w);
        sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, test_case.rho);
        
    else
        % Heavy-tailed noise case
        params = params_heavytail(test_case.pi, test_case.lambda);
        params.scenario = sprintf('robust_%s_pi%.2f_lambda%d', ...
            test_case.type, test_case.pi, test_case.lambda);
        
        dynamics = RobotDynamics(params.dt, params.sigma_w);
        
        gps_noise = MixtureNoise(params.sigma_gps^2 * eye(2), test_case.pi, test_case.lambda);
        odom_noise = MixtureNoise(params.sigma_odom^2 * eye(2), test_case.pi, test_case.lambda);
        sensors = {GPSSensor(gps_noise), OdometrySensor(odom_noise)};
    end
    
    % Run with robust filter
    runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials);
    results_robust = runner.run('robust');
    
    % Also run standard filter for comparison
    results_standard = runner.run('standard');
    
    % Save results
    save(sprintf('results/%s.mat', params.scenario), ...
        'results_robust', 'results_standard', 'params');
    
    % Generate comparison plots
    plot_robust_comparison(results_standard, results_robust, params);
    
    fprintf('Completed robust filter on %s\n', test_case.type);
end

fprintf('\nAll robust filter experiments complete!\n');