% RUN_VIDEO_BATCH Generates videos for all 4 filters (KF, EKF, RobustKF, RobustEKF).
%
% Two modes:
% 1. Generate videos for ONE scenario + ONE trajectory (faster)
% 2. Generate videos for ONE scenario + ALL trajectories (comprehensive)

clear;
close all;

setup_paths();

% ========== USER CONFIGURATION ==========
% Choose your mode below

% MODE 1: Single Trajectory (4 videos)
% Uncomment this block to generate videos for one trajectory only
MODE = 'SINGLE_TRAJECTORY';
SCENARIO_LABEL = 'baseline_gaussian';
TRAJECTORY = 'Circular';

% MODE 2: All Trajectories (16 videos)
% Uncomment this block to generate videos for all trajectories
% MODE = 'ALL_TRAJECTORIES';
% SCENARIO_LABEL = 'baseline_gaussian';

VIDEO_FOLDER = 'video_exports';

% ========== SCENARIO OPTIONS ==========
% Baseline: 'baseline_gaussian'
% Correlated: 'correlated_rho0.3', 'correlated_rho0.5', 'correlated_rho0.7', 'correlated_rho0.9'
% Heavy-tail: 'heavytail_pi0.05_lambda10', 'heavytail_pi0.05_lambda10', 'heavytail_pi0.10_lambda10'
%             (change lambda to 5, 10, 20, etc. as needed)

% ========== TRAJECTORY OPTIONS ==========
% 'Circular' - Simple circular path
% 'Figure8' - Figure-8 pattern
% 'Spiral' - Expanding spiral
% 'HighCurvature' - Sharp turns (exposes EKF limitations, ideal for RobustEKF comparison)

% ========== END USER CONFIGURATION ==========

fprintf('=== Batch Video Generation ===\n');
fprintf('Mode: %s\n', MODE);
fprintf('Scenario: %s\n', SCENARIO_LABEL);

% Run based on selected mode
switch MODE
    case 'SINGLE_TRAJECTORY'
        fprintf('Trajectory: %s\n', TRAJECTORY);
        fprintf('Expected: 4 videos (KF, EKF, RobustKF, RobustEKF)\n');
        fprintf('Output folder: %s/%s/%s/\n\n', VIDEO_FOLDER, SCENARIO_LABEL, TRAJECTORY);
        VideoBatchRunner.generateVideosForAllFilters(SCENARIO_LABEL, TRAJECTORY, VIDEO_FOLDER);
        
    case 'ALL_TRAJECTORIES'
        fprintf('Trajectories: Circular, Figure8, Spiral, HighCurvature\n');
        fprintf('Expected: 16 videos (4 filters × 4 trajectories)\n');
        fprintf('Output folder: %s/%s/{Circular,Figure8,Spiral,HighCurvature}/\n\n', VIDEO_FOLDER, SCENARIO_LABEL);
        VideoBatchRunner.generateVideosForScenario(SCENARIO_LABEL, VIDEO_FOLDER);
        
    otherwise
        error('Invalid MODE. Choose ''SINGLE_TRAJECTORY'' or ''ALL_TRAJECTORIES''.');
end

fprintf('All videos saved to: %s\n', VIDEO_FOLDER);
fprintf('Done!\n');
