% RUN_CORRELATED Runs the correlated noise scenarios, sweeping the correlation coefficient rho.
% This demonstrates how filter inconsistency increases with higher correlation.

clear;
close all;

setup_paths();

fprintf('=== Running Correlated Noise Sweep Experiments ===\n');

% Test multiple correlation values (Rho = 0 is equivalent to baseline noise assumption)
rho_values = [0.3, 0.5, 0.7, 0.9];

for i = 1:length(rho_values)
    rho = rho_values(i);
    scenario_label = sprintf('correlated_rho%.1f', rho);
    
    % Run all 4 filter types for this correlation value
    run_all_experiments_single(scenario_label, @(r) params_correlated(rho));
end

fprintf('\nAll correlated sweep experiments complete!\n');