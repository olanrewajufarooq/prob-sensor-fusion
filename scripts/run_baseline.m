% RUN_BASELINE Runs the core baseline scenario using the helper runner.
% This demonstrates the failure of KF due to non-linearity, 
% and the optimal performance of EKF under ideal conditions.

clear;
close all;

setup_paths();

% Define the scenario
scenario_label = 'baseline_gaussian';
params_func = @params_baseline;

% Run all 4 filter types for the baseline scenario
run_all_experiments_single(scenario_label, params_func);

fprintf('Baseline experiment complete. See output plots for EKF vs KF comparison.\n');