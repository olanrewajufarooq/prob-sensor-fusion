% Script for baseline experiment

%% Setup
clear; close all;

fprintf("===== Running Baseline Experiment =====\n");

setup_paths();

%% Get Parameters
params = params_baseline();

%% Create Dynamics
dynamics = RobotDynamics(params.dt, params.sigma_w);

%% Create sensors with independent Gaussian noise
gps_noise = GaussianNoise(params.sigma_gps^2 * eye(2));
odom_noise = GaussianNoise(params.sigma_odom^2 * eye(2));

sensors = {GPSSensor(gps_noise), OdometrySensor(odom_noise)};

%% Run experiment
runner = ExperimentRunner(dynamics, sensors, params.T, params.N_trials);
results = runner.run('standard');

%% Save results
save(sprintf('results/%s.mat', params.scenario), 'results', 'params');

%% Generate plots
plot_scenario_results(results, params, 'baseline');

fprintf('Baseline experiment complete!\n');