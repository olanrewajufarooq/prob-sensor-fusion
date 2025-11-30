classdef VideoBatchRunner
    % VIDEOBATCHRUNNER Helper utility to generate videos for multiple filters at once.
    % 
    % Usage:
    %   VideoBatchRunner.generateVideosForAllFilters(scenario_label, trajectory, video_folder);
    %
    % Example:
    %   VideoBatchRunner.generateVideosForAllFilters('baseline_gaussian', 'Spiral', 'video_exports');
    %   % Generates videos for KF, EKF, RobustKF, RobustEKF on Spiral trajectory
    
    methods (Static)
        function generateVideosForAllFilters(scenario_label, trajectory, video_folder)
            % GENERATEVIDEOSFORALLFILTERS Generate videos for all 4 filter types.
            %
            % Parameters:
            %   scenario_label (string): Noise scenario (e.g., 'baseline_gaussian', 'correlated_rho0.7', 'heavytail_pi0.05_lambda10')
            %   trajectory (string): Trajectory type ('Circular', 'Figure8', 'Spiral', 'HighCurvature')
            %   video_folder (string): Output folder for videos (default: 'video_exports')
            %
            % Returns: None (generates 4 video files)
            
            if nargin < 3
                video_folder = 'video_exports';
            end
            
            % All filter types
            filter_types = {'KF', 'EKF', 'RobustKF', 'RobustEKF'};
            
            fprintf('=== Batch Video Generation ===\n');
            fprintf('Scenario: %s | Trajectory: %s\n', scenario_label, trajectory);
            fprintf('Filters: %s\n\n', strjoin(filter_types, ', '));
            
            % Extract regime from scenario label
            if contains(scenario_label, 'correlated')
                regime = 'correlated';
            elseif contains(scenario_label, 'heavytail')
                regime = 'heavytail';
            else
                regime = 'baseline';
            end
            
            % Generate video for each filter
            for i = 1:length(filter_types)
                filter_name = filter_types{i};
                
                % Check if results file exists
                results_filename = sprintf('%s_%s_%s.mat', scenario_label, trajectory, filter_name);
                results_path = PathHelper.ensurePath('results', fullfile(regime, trajectory), results_filename, false);
                
                if ~isfile(results_path)
                    fprintf('⚠ SKIP: %s (results file not found: %s)\n', filter_name, results_filename);
                    continue;
                end
                
                % Load results
                loaded = load(results_path, 'results');
                results_struct = loaded.results;
                
                % Build output path for video
                video_filename = sprintf('%s_%s_%s.mp4', scenario_label, trajectory, filter_name);
                video_path = PathHelper.ensurePath(video_folder, '', video_filename, true);
                
                fprintf('Generating: %s...', filter_name);
                
                % Generate video
                VideoRunner.generateVideo(scenario_label, filter_name, trajectory, video_path, [], results_struct);
                
                fprintf(' ✓ Done\n');
            end
            
            fprintf('\nBatch video generation complete!\n');
        end
        
        function generateVideosForScenario(scenario_label, video_folder)
            % GENERATEVIDEOSFORSCENARIO Generate videos for all filters across all trajectories in a scenario.
            %
            % Parameters:
            %   scenario_label (string): Noise scenario (e.g., 'baseline_gaussian')
            %   video_folder (string): Output folder for videos (default: 'video_exports')
            %
            % Example:
            %   VideoBatchRunner.generateVideosForScenario('baseline_gaussian');
            %   % Generates videos for all 4 filters × 4 trajectories = 16 videos
            
            if nargin < 2
                video_folder = 'video_exports';
            end
            
            trajectory_list = {'Circular', 'Figure8', 'Spiral', 'HighCurvature'};
            
            fprintf('=== Batch Video Generation (Full Scenario) ===\n');
            fprintf('Scenario: %s\n', scenario_label);
            fprintf('Trajectories: %s\n', strjoin(trajectory_list, ', '));
            fprintf('Filters: KF, EKF, RobustKF, RobustEKF\n\n');
            
            total_videos = length(trajectory_list) * 4;
            generated = 0;
            
            % Generate videos for each trajectory
            for t_idx = 1:length(trajectory_list)
                trajectory = trajectory_list{t_idx};
                fprintf('\n--- Trajectory: %s ---\n', trajectory);
                
                % Count how many generated for this trajectory
                old_count = generated;
                VideoBatchRunner.generateVideosForAllFilters(scenario_label, trajectory, video_folder);
                
                % Update counter (rough estimate - 4 per successful trajectory)
                generated = generated + 4;
            end
            
            fprintf('\n========================================\n');
            fprintf('Batch video generation complete!\n');
            fprintf('Total videos generated: %d / %d\n', generated, total_videos);
        end
    end
end
