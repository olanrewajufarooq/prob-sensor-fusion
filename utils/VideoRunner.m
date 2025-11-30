classdef VideoRunner
    properties (Constant)
        FRAME_RATE = 20;
        VIDEO_QUALITY = 75;
    end

    methods (Static)
        function generateVideo(scenario_label, filter_name, trajectory_type, video_path, trajectory_generator, results)

            if nargin < 6
                results = [];
            end

            has_results = ~isempty(results) && isfield(results, 'traj_true') && ~isempty(results.traj_true);
            if ~has_results
                error('No stored results provided for %s. Please run the simulation first to generate results.', scenario_label);
            end

            x_history = results.traj_true{1};
            x_hat_history = results.traj_hat{1};
            if isfield(results, 'P_history') && ~isempty(results.P_history)
                P_history = results.P_history{1};
            else
                P_history = cell(1, size(x_history, 2));
            end
            if isfield(results, 'u_history') && ~isempty(results.u_history)
                u_history = results.u_history{1};
            else
                u_history = zeros(2, size(x_history, 2)); % controls: [v; omega]
            end
            T_total = size(x_history, 2);
            if isfield(results, 'dt') && ~isempty(results.dt)
                dt_vis = results.dt;
            else
                dt_vis = 0.1;
            end
            if isfield(results, 'trajectory_type') && ~isempty(results.trajectory_type)
                trajectory_type = results.trajectory_type;
            elseif nargin < 3 || isempty(trajectory_type)
                trajectory_type = VideoRunner.inferTrajectoryFromLabel(scenario_label);
            end

            % Derive noise_name from scenario_label
            noise_name = VideoRunner.deriveNoiseName(scenario_label);

            if exist(video_path, 'file')
                fprintf('Video already exists at %s. Skipping rendering.\n', video_path);
                return;
            end

            PathHelper.ensurePath(fileparts(video_path), '', '', true);
            vid_writer = VideoWriter(video_path, 'MPEG-4');
            vid_writer.FrameRate = VideoRunner.FRAME_RATE;
            vid_writer.Quality = VideoRunner.VIDEO_QUALITY;
            open(vid_writer);

            figure('Position', [100, 100, 800, 700], 'Color', 'w');
            ax = gca;
            title_str = sprintf('Localization: %s (Noise: %s)', filter_name, noise_name);
            title(ax, title_str);
            xlabel(ax, 'X Position [m]');
            ylabel(ax, 'Y Position [m]');
            axis equal;
            grid on;
            hold on;

            % Preplot full true trajectory
            true_full = plot(ax, x_history(1, :), x_history(2, :), ':', 'LineWidth', 1, 'Color', [0 0.5 0], 'DisplayName', 'True Path (Full)');

            for t = 1:T_total
                % Clear previous evolving path and dynamic elements
                delete(findobj(ax, 'DisplayName', 'Est. Path (evolving)'));
                delete(findobj(ax, 'DisplayName', 'Vehicle Position'));
                delete(findobj(ax, 'DisplayName', 'Direction Arrow'));

                % Replot evolving estimated path
                est_evolv = plot(ax, x_hat_history(1, 1:t), x_hat_history(2, 1:t), '-', 'LineWidth', 2, 'Color', [1 0 0 1], 'DisplayName', 'Est. Path (evolving)');

                % Current position with circle marker
                plot(ax, x_hat_history(1, t), x_hat_history(2, t), 'ro', 'MarkerSize', 8, 'DisplayName', 'Vehicle Position');
                
                % Direction arrow from position
                theta = x_hat_history(3, t);
                arrow_len = 0.5;
                quiver(ax, x_hat_history(1, t), x_hat_history(2, t), ...
                       arrow_len*cos(theta), arrow_len*sin(theta), ...
                       'Color', 'b', 'LineWidth', 1.5, 'MaxHeadSize', 0.4, 'AutoScale', 'off', 'DisplayName', 'Direction Arrow');

                % Update title with time
                title(ax, sprintf('%s | Time: %.1fs', title_str, t * dt_vis));
                
                % Add legend
                legend(ax, [true_full, est_evolv], ...
                       {'True Path (Full)', 'Est. Path (Evolving)'}, ...
                       'Location', 'best');
                
                frame = getframe(gcf);
                writeVideo(vid_writer, frame);
            end

            close(vid_writer);
            close(gcf);
            fprintf('Video successfully saved to: %s\n', video_path);
        end
        
        function trajectory_type = inferTrajectoryFromLabel(label)
            if contains(label, 'Figure8')
                trajectory_type = 'Figure8';
            elseif contains(label, 'Spiral')
                trajectory_type = 'Spiral';
            else
                trajectory_type = 'Circular';
            end
        end

        function noise_name = deriveNoiseName(scenario_label)
            % Derive human-readable noise description from scenario label
            if contains(scenario_label, 'correlated')
                if contains(scenario_label, 'rho0.9')
                    noise_name = 'Correlated Gaussian (ρ=0.9)';
                elseif contains(scenario_label, 'rho0.7')
                    noise_name = 'Correlated Gaussian (ρ=0.7)';
                elseif contains(scenario_label, 'rho0.5')
                    noise_name = 'Correlated Gaussian (ρ=0.5)';
                elseif contains(scenario_label, 'rho0.3')
                    noise_name = 'Correlated Gaussian (ρ=0.3)';
                else
                    noise_name = 'Correlated Gaussian';
                end
            elseif contains(scenario_label, 'heavytail')
                if contains(scenario_label, 'pi0.1')
                    noise_name = 'Heavy-Tail Mixture (π=0.1)';
                elseif contains(scenario_label, 'pi0.05')
                    noise_name = 'Heavy-Tail Mixture (π=0.05)';
                elseif contains(scenario_label, 'pi0.01')
                    noise_name = 'Heavy-Tail Mixture (π=0.01)';
                else
                    noise_name = 'Heavy-Tail Mixture';
                end
            else
                % Baseline
                noise_name = 'Standard Gaussian';
            end
        end

    end
end
