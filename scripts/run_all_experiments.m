% RUN_ALL_EXPERIMENTS Runs Monte Carlo simulations for the core analysis scenarios 
% (Baseline, Correlated, Heavy-Tail) and generates the final comparison plots.
% This script runs all four filters (KF, RobustKF, EKF, RobustEKF) on the
% designated core scenarios.

clear;
close all;

% 1. Setup paths (Updated for new folder structure)
setup_paths;

% 2. Run baseline
fprintf('\n--- Running Baseline ---\n');
run_baseline;

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