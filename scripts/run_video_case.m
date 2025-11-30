clear;
close all;

setup_paths(); % add repo paths for scripts and utils

VIDEO_FOLDER = 'video_exports';
SCENARIO_LABEL = 'correlated_rho0.7';
FILTER_TYPE   = 'RobustEKF';
TRAJECTORY    = 'Figure8';
RESULTS_MAT   = ''; % path to .mat containing 'results'

% Build output path (creates folder) and short-circuit if results file missing
video_filename = sprintf('%s_%s_%s_%s.mp4', SCENARIO_LABEL, TRAJECTORY, FILTER_TYPE, datestr(now,'yymmddHHMM'));
video_path = PathHelper.ensurePath(VIDEO_FOLDER, '', video_filename, true);

fprintf('=== Generating Video Visualization ===\n');
fprintf('Scenario: %s | Filter: %s | Trajectory: %s\n', SCENARIO_LABEL, FILTER_TYPE, TRAJECTORY);

if isempty(RESULTS_MAT) || ~isfile(RESULTS_MAT)
    error('RESULTS_MAT not set or file missing. Run the simulation to generate results before making a video.');
end

loaded = load(RESULTS_MAT, 'results'); % load stored trial data
results_struct = loaded.results;
fprintf('Loaded stored results from %s (replaying trial 1).\n', RESULTS_MAT);

VideoRunner.generateVideo(SCENARIO_LABEL, FILTER_TYPE, TRAJECTORY, video_path, [], results_struct);

fprintf('Video generation routine finished.\n');
