% RUN_HEAVYTAIL Runs the heavy-tailed noise scenarios, sweeping outlier probability (pi) 
% and outlier magnitude (lambda).
% This demonstrates how filter performance degrades with increasing non-Gaussianity.

clear;
close all;

setup_paths();

fprintf('=== Running Heavy-Tailed Noise Sweep Experiments ===\n');

% Test configurations: {pi_outlier, lambda}
configs = {
    {0.05, 5}, 	 % Rare, moderate outliers
    {0.05, 10}, 	% Rare, severe outliers
    {0.10, 5}, 	 % Common, moderate outliers
    {0.10, 10} 	 % Common, severe outliers
};

for i = 1:length(configs)
    pi_outlier = configs{i}{1};
    lambda = configs{i}{2};
    
    scenario_label = sprintf('heavytail_pi%.2f_lambda%d', pi_outlier, lambda);
    
    % Run all 4 filter types for this heavy-tail configuration
    run_all_experiments_single(scenario_label, @(p, l) params_heavytail(pi_outlier, lambda));
end

fprintf('\nAll heavy-tailed sweep experiments complete!\n');