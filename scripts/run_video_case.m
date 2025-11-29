% RUN_VIDEO_CASE Executes a single trial and generates a live visualization video.

clear;
close all;

setup_paths();

% --- CONFIGURATION ---
% Define the scenario and filter you want to visualize

VIDEO_FOLDER = 'video_exports';
SCENARIO_LABEL = 'correlated_rho0.7'; % e.g., 'baseline_gaussian', 'correlated_rho0.7', 'heavytail_pi0.05_lambda10'
FILTER_TYPE = 'RobustEKF';           % e.g., 'KF', 'EKF', 'RobustEKF'

% Define output path
video_filename = sprintf('%s_%s_%s.mp4', SCENARIO_LABEL, FILTER_TYPE, datestr(now,'yymmddHHMM'));
video_path = fullfile(VIDEO_FOLDER, video_filename);

fprintf('=== Generating Video Visualization ===\n');
fprintf('Scenario: %s | Filter: %s\n', SCENARIO_LABEL, FILTER_TYPE);

% Call the static method in VideoRunner to generate the output
VideoRunner.generateVideo(SCENARIO_LABEL, FILTER_TYPE, 'plots', video_path);

fprintf('Video generation routine finished.\n');