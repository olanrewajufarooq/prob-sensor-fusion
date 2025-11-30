clear;
close all;

setup_paths(); % add repo paths for scripts and utils

% ========== USER CONFIGURATION ==========
% Modify these parameters to generate different videos

% Output folder for videos
VIDEO_FOLDER = 'video_exports';

% SCENARIO_LABEL: Noise characteristics
% Baseline (Standard Gaussian):
%   - 'baseline_gaussian'
% Correlated Gaussian (choose correlation coefficient):
%   - 'correlated_rho0.3'
%   - 'correlated_rho0.5'
%   - 'correlated_rho0.7'
%   - 'correlated_rho0.9'
% Heavy-tail Mixture (choose outlier probability):
%   - 'heavytail_pi0.01_lambda5'
%   - 'heavytail_pi0.05_lambda5'
%   - 'heavytail_pi0.10_lambda5'
SCENARIO_LABEL = 'heavytail_pi0.05_lambda5';

% FILTER_TYPE: Filtering algorithm
%   - 'KF'          (Kalman Filter - linear only)
%   - 'EKF'         (Extended Kalman Filter - nonlinear)
%   - 'RobustKF'    (Robust Kalman Filter - outlier detection)
%   - 'RobustEKF'   (Robust Extended Kalman Filter - nonlinear + robust)
FILTER_TYPE   = 'RobustKF';

% TRAJECTORY: Motion path type
%   - 'Circular'       (Circular path)
%   - 'Figure8'        (Figure-8 pattern)
%   - 'Spiral'         (Spiral trajectory)
%   - 'HighCurvature'  (Sharp turns - exposes EKF limitations, ideal for RobustEKF)
TRAJECTORY    = 'Circular';

% ========== END USER CONFIGURATION ==========

% Determine results file path automatically
% Expected format: results/<regime>/<trajectory>/<scenario_label>_<trajectory>_<filter_type>.mat
% E.g., results/baseline/Circular/baseline_gaussian_Circular_RobustEKF.mat

% Extract regime from scenario label (baseline, correlated, or heavytail)
if contains(SCENARIO_LABEL, 'correlated')
    regime = 'correlated';
elseif contains(SCENARIO_LABEL, 'heavytail')
    regime = 'heavytail';
else
    regime = 'baseline';  % default
end

% Construct expected results filename
results_filename = sprintf('%s_%s_%s.mat', SCENARIO_LABEL, TRAJECTORY, FILTER_TYPE);
RESULTS_MAT = PathHelper.ensurePath('results', fullfile(regime, TRAJECTORY), results_filename, false);

fprintf('=== Generating Video Visualization ===\n');
fprintf('Scenario: %s | Filter: %s | Trajectory: %s\n', SCENARIO_LABEL, FILTER_TYPE, TRAJECTORY);

% Check if results file exists
if ~isfile(RESULTS_MAT)
    fprintf('\nERROR: Results file not found!\n');
    fprintf('Expected path: %s\n\n', RESULTS_MAT);
    fprintf('Please run the simulation first to generate the results file.\n');
    fprintf('Example: run_baseline  or  run_correlated  or  run_heavytail\n\n');
    fprintf('Alternatively, manually run:\n');
    fprintf('  run_all_experiments_single(''%s'', @() params_%s())\n', SCENARIO_LABEL, regime);
    error('Results file not found: %s', RESULTS_MAT);
end

loaded = load(RESULTS_MAT, 'results'); % load stored trial data
results_struct = loaded.results;
fprintf('Loaded stored results from %s (replaying trial 1).\n', RESULTS_MAT);

% Build output path (creates folder) for video
video_filename = sprintf('%s_%s_%s.mp4', SCENARIO_LABEL, TRAJECTORY, FILTER_TYPE);
video_path = PathHelper.ensurePath(VIDEO_FOLDER, '', video_filename, true);

VideoRunner.generateVideo(SCENARIO_LABEL, FILTER_TYPE, TRAJECTORY, video_path, [], results_struct);

fprintf('Video generation routine finished.\n');
fprintf('Video saved to: %s\n', video_path);
