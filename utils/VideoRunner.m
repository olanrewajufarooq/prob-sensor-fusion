classdef VideoRunner
    properties (Constant)
        FRAME_RATE = 10;
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

            if nargin < 5 || isempty(trajectory_generator)
                traj_gen = TrajectoryGenerator(dt_vis, T_total, trajectory_type);
            else
                traj_gen = trajectory_generator;
            end

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
            title_str = sprintf('Localization: %s on %s (Path: %s | Noise: %s)', ...
                                filter_name, strrep(scenario_label, '_', ' '), trajectory_type, noise_name);
            title(ax, title_str);
            xlabel(ax, 'X Position [m]');
            ylabel(ax, 'Y Position [m]');
            axis equal;
            grid on;
            hold on;

            T_path = min(T_total, size(traj_gen.target_path, 1));
            x_desired_path = traj_gen.target_path(1:T_path, :)';

            for t = 1:T_total
                cla(ax);

                desired_handle = plot(ax, x_desired_path(1,:), x_desired_path(2,:), 'k:', 'LineWidth', 0.5, 'DisplayName', 'Desired Path');
                true_handle = plot(ax, x_history(1, 1:t), x_history(2, 1:t), 'g-', 'LineWidth', 1, 'DisplayName', 'True Path');
                est_handle = plot(ax, x_hat_history(1, 1:t), x_hat_history(2, 1:t), 'r--', 'LineWidth', 1, 'DisplayName', 'Estimated Path');

                plot(ax, x_history(1, t), x_history(2, t), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6, 'DisplayName', 'True Vehicle');

                plot(ax, x_hat_history(1, t), x_hat_history(2, t), 'rd', 'MarkerFaceColor', 'r', 'MarkerSize', 7, 'DisplayName', 'Estimated Position');
                
                if t <= numel(P_history) && ~isempty(P_history{t})
                    Plotter.plotCovarianceEllipse(x_hat_history(1:2, t), P_history{t}(1:2, 1:2), 2);
                end

                v_mag = x_history(4, t);
                theta = x_history(3, t);
                vel_handle = quiver(ax, x_history(1, t), x_history(2, t), v_mag*cos(theta), v_mag*sin(theta), ...
                                    'Color', 'b', 'LineWidth', 1, 'MaxHeadSize', 2, 'AutoScale', 'off', 'DisplayName', 'Velocity');

                text(ax, min(x_desired_path(1,:)), max(x_desired_path(2,:)) + 5, ...
                    sprintf('Time: %.1fs\nSpeed Cmd: %.2f m/s\nTurn Cmd: %.2f rad/s', ...
                            t * dt_vis, u_history(1, min(t, size(u_history,2))), u_history(2, min(t, size(u_history,2)))), ...
                    'BackgroundColor', [1 1 1 0.7], 'FontSize', 8, 'EdgeColor', 'k', 'VerticalAlignment', 'top');

                title(ax, sprintf('%s (Time: %.1fs)', title_str, t * dt_vis));
                legend(ax, [desired_handle, true_handle, est_handle, vel_handle], {'Desired Path', 'True Path', 'Estimated Path', 'Velocity'}, 'Location', 'best');
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
    end
end
