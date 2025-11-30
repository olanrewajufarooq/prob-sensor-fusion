function run_all_experiments_single(base_scenario_label, params_func)

filter_types = {'KF', 'RobustKF', 'EKF', 'RobustEKF'};

params = params_func();
dynamics = RobotDynamics(params.dt, params.Q);

% Sweep across these trajectories for the scenario
trajectory_list = {'Circular', 'Figure8', 'Spiral', 'HighCurvature'};

if contains(base_scenario_label, 'correlated')
    subfolder_name = 'correlated';
    sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, params.rho);
    noise_name = 'Correlated Gaussian';
elseif contains(base_scenario_label, 'heavytail')
    subfolder_name = 'heavytail';
    % GPS sensor with mixture/heavy-tail noise
    R_gps = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model_gps = MixtureNoise(R_gps, params.pi_outlier, params.lambda);
    gps_sensor = GPSSensor(noise_model_gps);
    
    % Odometry sensor with standard Gaussian noise
    R_odom = diag([params.sigma_odom^2, params.sigma_odom^2]);
    noise_model_odom = GaussianNoise(R_odom);
    odom_sensor = OdometrySensor(noise_model_odom);
    
    sensors = {gps_sensor, odom_sensor};
    noise_name = 'Mixture/Heavy-Tail (GPS) + Gaussian (Odometry)';
else 
    subfolder_name = 'baseline';
    % GPS sensor with standard Gaussian noise
    R_gps = diag([params.sigma_gps^2, params.sigma_gps^2]);
    noise_model_gps = GaussianNoise(R_gps);
    gps_sensor = GPSSensor(noise_model_gps);
    
    % Odometry sensor with standard Gaussian noise
    R_odom = diag([params.sigma_odom^2, params.sigma_odom^2]);
    noise_model_odom = GaussianNoise(R_odom);
    odom_sensor = OdometrySensor(noise_model_odom);
    
    sensors = {gps_sensor, odom_sensor};
    noise_name = 'Standard Gaussian (Both Sensors)';
end

    for t_idx = 1:length(trajectory_list)
        current_trajectory_type = trajectory_list{t_idx};
        
        scenario_label = sprintf('%s_%s', base_scenario_label, current_trajectory_type);
        
        % Stable folder creation for outputs
        plot_subfolder = PathHelper.ensurePath('plots', fullfile(subfolder_name, current_trajectory_type), '', true);
        results_subfolder = PathHelper.ensurePath('results', fullfile(subfolder_name, current_trajectory_type), '', true);

        runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials, params.pid_params, current_trajectory_type);

        fprintf('\nRunning Scenario: %s (Trajectory: %s)\n', base_scenario_label, current_trajectory_type);
        
        for f = 1:length(filter_types)
            filter_name = filter_types{f};
            
            results_filename = fullfile(results_subfolder, sprintf('%s_%s.mat', scenario_label, filter_name));
            
            if exist(results_filename, 'file')
                fprintf('  Skipping %s (Results already exist).\n', filter_name);
                load(results_filename, 'results');
            else
                fprintf('  Running Filter: %s (Noise: %s)...\n', filter_name, noise_name);
                results = runner.run(filter_name);
                save(results_filename, 'results', 'params', 'filter_name');
            end

         nees_mean = mean(cell2mat(arrayfun(@(i) Metrics.computeNEES(results.errors(:,:,i), results.P_history{i}), 1:params.N_trials, 'UniformOutput', false)'), 1);
         
         x_true_trajectory = runner.Trajectory.getDesiredState(1:params.T);
         x_est_trajectory = x_true_trajectory - results.errors(:,:,1); 
         
         Plotter.plotTrajectory(x_true_trajectory, x_est_trajectory, ...
                                results.P_history{1}, scenario_label, filter_name, plot_subfolder);
         Plotter.plotNEES(nees_mean, dynamics.nx, scenario_label, filter_name, plot_subfolder);
         Plotter.plotErrorHistogram(results.errors, scenario_label, filter_name, plot_subfolder);
    end

    fprintf('Completed all filters for scenario: %s.\n', scenario_label);
end

end
