classdef VideoRunner
    % VIDEORUNNER Runs a single simulation trial and generates a video visualization 
    % showing the vehicle's movement, estimated state, and uncertainty ellipse.
    
    properties (Constant)
        FRAME_RATE = 10; % Frames per second
        VIDEO_QUALITY = 75; % Quality percentage
    end

    methods (Static)
        function generateVideo(scenario_label, filter_name, plot_subfolder, video_path)
            
            % --- 1. SETUP ENVIRONMENT ---
            
            % Determine parameter function based on label
            if contains(scenario_label, 'correlated')
                params = params_correlated(0.7);
                noise_name = 'Correlated Gaussian';
                sensors = create_correlated_sensors(params.sigma_gps, params.sigma_odom, params.rho);
            elseif contains(scenario_label, 'heavytail')
                params = params_heavytail(0.05, 10);
                noise_name = 'Mixture/Heavy-Tail';
                R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
                noise_model = MixtureNoise(R_nominal, params.pi_outlier, params.lambda);
                sensors = {GPSSensor(noise_model)};
            else 
                params = params_baseline();
                noise_name = 'Standard Gaussian';
                R_nominal = diag([params.sigma_gps^2, params.sigma_gps^2]);
                noise_model = GaussianNoise(R_nominal);
                sensors = {GPSSensor(noise_model)};
            end
            
            dynamics = RobotDynamics(params.dt, params.Q);
            runner = ExperimentRunner(dynamics, sensors, params.T, 1, params.pid_params, 'Circular'); % Fixed to Circular for video demo
            
            % --- 2. INITIALIZE FILTER (Trial 1 setup) ---
            x_true = [0; 0; 0; 0]; 
            x0 = x_true + mvnrnd(zeros(dynamics.nx,1), diag([1, 1, 0.1, 0.1]))';
            P0 = diag([1.0, 1.0, 0.5, 0.5]);
            
            switch filter_name
                case 'KF', filter = KalmanFilter(dynamics, sensors, x0, P0);
                case 'RobustKF', filter = RobustKalmanFilter(dynamics, sensors, x0, P0);
                case 'EKF', filter = ExtendedKalmanFilter(dynamics, sensors, x0, P0);
                case 'RobustEKF', filter = RobustEKF(dynamics, sensors, x0, P0);
                otherwise, error('Unknown filter type: %s', filter_name);
            end

            % Initialize Video Writer
            if ~exist(fileparts(video_path), 'dir')
                mkdir(fileparts(video_path));
            end
            
            vid_writer = VideoWriter(video_path, 'MPEG-4');
            vid_writer.FrameRate = VideoRunner.FRAME_RATE;
            vid_writer.Quality = VideoRunner.VIDEO_QUALITY;
            open(vid_writer);

            % Setup Figure and Axes
            figure('Position', [100, 100, 800, 700], 'Color', 'w');
            ax = gca;
            title_str = sprintf('Localization: %s on %s (Noise: %s)', filter_name, strrep(scenario_label, '_', ' '), noise_name);
            title(ax, title_str);
            xlabel(ax, 'X Position [m]');
            ylabel(ax, 'Y Position [m]');
            axis equal;
            grid on;
            hold on;

            % Plot desired trajectory once
            x_desired_path = runner.Trajectory.getDesiredState(1:params.T);
            plot(ax, x_desired_path(1,:), x_desired_path(2,:), 'k:', 'LineWidth', 0.5, 'DisplayName', 'Desired Path');

            % Initial state and control
            u_t_prev = zeros(dynamics.nu, 1);
            
            % --- 3. MAIN SIMULATION AND RENDERING LOOP ---
            
            for t = 1:params.T
                x_desired = runner.Trajectory.getDesiredState(t);
                
                % 1. Compute Control Input
                u_t = runner.Controller.computeControl(filter.x_hat, x_desired, dynamics.dt); 
                
                % 2. Filter Prediction
                filter = filter.predict(u_t_prev); 
                
                % 3. Generate measurements and propagate TRUE state
                w = mvnrnd(zeros(dynamics.nx, 1), dynamics.Q)'; 
                x_true = dynamics.propagate(x_true, u_t, w);
                
                for s = 1:length(sensors)
                    z = sensors{s}.measure(x_true);
                    filter = filter.update(z, s); 
                end
                
                % --- RENDERING ---
                
                cla(ax); % Clear axes for dynamic plot
                
                % Replot static and history elements
                plot(ax, x_desired_path(1,:), x_desired_path(2,:), 'k:', 'LineWidth', 0.5, 'DisplayName', 'Desired Path');
                
                % Plot history (True and Estimated)
                if t > 1
                    % True History (Green line)
                    plot(ax, [x_history(1,:), x_true(1)], [x_history(2,:), x_true(2)], 'g-', 'LineWidth', 1, 'DisplayName', 'True Path');
                    % Estimated History (Red dashed line)
                    plot(ax, [x_hat_history(1,:), filter.x_hat(1)], [x_hat_history(2,:), filter.x_hat(2)], 'r--', 'LineWidth', 1, 'DisplayName', 'Estimated Path');
                end

                % Plot current TRUE vehicle position (Green marker)
                plot(ax, x_true(1), x_true(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 6, 'DisplayName', 'True Vehicle');

                % Plot current ESTIMATED vehicle position (Red marker)
                plot(ax, filter.x_hat(1), filter.x_hat(2), 'rd', 'MarkerFaceColor', 'r', 'MarkerSize', 7, 'DisplayName', 'Estimated Position');
                
                % Plot 2-sigma Uncertainty Ellipse (Blue)
                Plotter.plotCovarianceEllipse(filter.x_hat(1:2), filter.P(1:2, 1:2), 2); % Needs helper function definition

                % Display Control Command (u_t) and Time
                text(ax, x_true(1) + 5, x_true(2) + 5, ...
                     sprintf('Time: %.1fs\nSpeed Cmd: %.2f m/s\nTurn Cmd: %.2f rad/s', ...
                             t * dynamics.dt, u_t(1), u_t(2)), ...
                     'BackgroundColor', [1 1 1 0.7], 'FontSize', 8, 'EdgeColor', 'k');

                % --- Frame Capture ---
                title(ax, sprintf('%s (Time: %.1fs)', title_str, t * dynamics.dt));
                legend(ax, 'Location', 'best');
                frame = getframe(gcf);
                writeVideo(vid_writer, frame);

                % --- History Update ---
                x_history(:, t) = x_true;
                x_hat_history(:, t) = filter.x_hat;
                u_t_prev = u_t;
            end

            % --- 4. CLEAN UP ---
            close(vid_writer);
            close(gcf);
            fprintf('Video successfully saved to: %s\n', video_path);
        end
    end
end