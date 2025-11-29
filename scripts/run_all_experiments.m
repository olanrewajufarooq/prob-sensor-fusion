% RUN_ALL_EXPERIMENTS Runs Monte Carlo simulations for the core analysis scenarios 
% (Baseline, Correlated, Heavy-Tail) and generates the final comparison plots.
% This script runs all four filters (KF, RobustKF, EKF, RobustEKF) on the
% designated core scenarios.

clc;
clear all;
close all;

% 1. Setup paths
setup_paths;

% 2. Define CORE ANALYSIS Scenarios (Scenario Label, Parameter Function)
% These are the key scenarios where we run all four filters for direct comparison.
core_scenarios = {
    'baseline_gaussian', @params_baseline;
    'correlated_rho0.7', @(~) params_correlated(0.7);
    'heavytail_pi0.05_lambda10', @(~) params_heavytail(0.05, 10);
};

for i = 1:length(core_scenarios)
    scenario_label = core_scenarios{i, 1};
    params_func = core_scenarios{i, 2};
    
    % Call the single runner helper function for all four filter types
    run_all_experiments_single(scenario_label, params_func);
end

% 3. Run all parameter sweep experiments (these scripts save their own results)
fprintf('\n--- Running Correlated Noise Sweep ---\n');
run_correlated;

fprintf('\n--- Running Heavy-Tail Noise Sweep ---\n');
run_heavytail;

fprintf('\n--- Running Robustness Parameter Delta Sweep ---\n');
run_robust;

% 4. Generate Comparative Plots using all saved results
fprintf('\n\n======================================================\n');
fprintf('  FINAL ANALYSIS: Generating Comparative Plots\n');
fprintf('======================================================\n');
compare_all;

fprintf('\nALL EXPERIMENTS COMPLETE. Results saved in "results" and plots in "plots" folder.\n');