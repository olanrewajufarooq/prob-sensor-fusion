% Generate comprehensive comparison plots across all scenarios
clear; clc; close all;
setup_paths();

fprintf('=== Generating Comprehensive Comparison Plots ===\n');

% Load all results
files = dir('results/*.mat');
all_results = struct();

for k = 1:numel(files)
    fname = files(k).name;
    fprintf('Loaded: %s\n', fname);

    % Load file
    data = load(fullfile('results', fname));

    % Use a safe field name (MATLAB struct field rules)
    [~, base, ~] = fileparts(fname);
    safe_name = matlab.lang.makeValidName(base);

    % The .mat files are assumed to contain either: results or 
    % results_robust. Both are already handled inside 
    % generate_comparison_plots
    all_results.(safe_name) = data;
end

% Generate comparison plots
generate_comparison_plots(all_results);

fprintf('\nComparison plots complete!\n');