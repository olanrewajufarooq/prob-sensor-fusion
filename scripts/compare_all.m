% COMPARE_ALL Loads all saved results and generates the comprehensive comparison plots.

clear all;
close all;

setup_paths();

fprintf('=== Loading and Generating Comprehensive Comparison Plots ===\n');

% Load all results
files = dir('results/*.mat');
all_results = struct();

% Group results by scenario
for k = 1:numel(files)
    fname = files(k).name;
    
    % The pattern captures the scenario name (Group 1) and the filter name (Group 2)
    % This handles results from KF, RobustKF, EKF, RobustEKF
    pattern = '(.+?)_(KF|RobustKF|EKF|RobustEKF)\.mat';
    tokens = regexp(fname, pattern, 'tokens');
    
    if ~isempty(tokens)
        scenario_name = tokens{1}{1};
        filter_name = tokens{1}{2};
        
        % Load data from the file
        data = load(fullfile('results', fname));
        
        if ~isfield(all_results, scenario_name)
            all_results.(scenario_name) = struct();
        end
        
        % Store the results structure under the filter name
        all_results.(scenario_name).(filter_name) = data.results;
    end
end

% The sweep scripts (run_robust) might save files with delta in the name,
% which may require manual grouping or using a different comparison function.
% Assuming generate_comparison_plots handles the grouped structure well.

% Generate comparison plots
generate_comparison_plots(all_results, 'plots');

fprintf('\nComparison plots complete!\n');