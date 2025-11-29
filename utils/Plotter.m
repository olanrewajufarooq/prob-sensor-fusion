classdef Plotter
    % PLOTTER Handles visualization of simulation results.
    
    methods (Static)
        function plotTrajectory(x_true, x_est, P_history, scenario_name, filter_name, plot_folder)
            % x_true: 4xT matrix, x_est: 4xT matrix, P_history: cell array
            
            % Ensure subfolder exists
            if ~exist(plot_folder, 'dir')
                mkdir(plot_folder);
            end

            figure('Position', [100, 100, 800, 600]);
            plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 2); hold on;
            plot(x_est(1,:), x_est(2,:), 'r--', 'LineWidth', 1.5);
            
            % Plot covariance ellipses every 10 steps for the position components (px, py)
            for t = 1:10:length(P_history)
                % Only use the top-left 2x2 block for position covariance
                Plotter.plotCovarianceEllipse(x_est(1:2,t), P_history{t}(1:2,1:2), 2);
            end
            
            xlabel('x position [m]');
            ylabel('y position [m]');
            legend('True Trajectory', 'Estimated Trajectory', '2\sigma Position Ellipse', 'Location', 'best');
            title(sprintf('Trajectory: %s - %s', strrep(scenario_name, '_', ' '), filter_name));
            grid on; axis equal;
            
            saveas(gcf, fullfile(plot_folder, sprintf('trajectory_%s_%s.png', scenario_name, filter_name)));
            close(gcf);
        end
        
        function plotCovarianceEllipse(mu, Sigma, nsigma)
            % Plots nsigma confidence ellipse for 2D Gaussian
            [V, D] = eig(Sigma);
            theta = linspace(0, 2*pi, 100);
            ellipse = nsigma * V * sqrt(D) * [cos(theta); sin(theta)];
            plot(mu(1) + ellipse(1,:), mu(2) + ellipse(2,:), 'b-', 'LineWidth', 0.5);
        end
        
        function plotNEES(nees_mean, nx, scenario_name, filter_name, plot_folder)
            % nees_mean: 1xT vector (averaged over trials)

            % Ensure subfolder exists
            if ~exist(plot_folder, 'dir')
                mkdir(plot_folder);
            end

            figure('Position', [100, 100, 800, 600]);
            plot(nees_mean, 'LineWidth', 1.5, 'DisplayName', 'Mean NEES'); hold on;
            yline(nx, 'r--', 'Expected NEES', 'LineWidth', 2);
            
            % Chi-Squared bounds for consistency check (using N_trials = 500)
            N_trials = 500; 
            alpha = 0.05;
            r1 = chi2inv(alpha/2, nx*N_trials) / N_trials;
            r2 = chi2inv(1 - alpha/2, nx*N_trials) / N_trials;
            
            yline(r1, 'k:', '95% Lower Bound');
            yline(r2, 'k:', '95% Upper Bound');
            
            xlabel('Time step');
            ylabel('NEES');
            title(sprintf('NEES Time Series: %s - %s', strrep(scenario_name, '_', ' '), filter_name));
            legend('Location', 'best');
            grid on;
            
            saveas(gcf, fullfile(plot_folder, sprintf('nees_%s_%s.png', scenario_name, filter_name)));
            close(gcf);
        end
        
        function plotErrorHistogram(errors, scenario_name, filter_name, plot_folder)
            % errors: nx x T x N_trials

            % Ensure subfolder exists
            if ~exist(plot_folder, 'dir')
                mkdir(plot_folder);
            end

            figure('Position', [100, 100, 800, 600]);
            
            % Focus on positional error magnitude (px, py)
            pos_errors_flat = reshape(errors(1:2, :, :), 2, []);
            error_norms = sqrt(sum(pos_errors_flat.^2, 1));
            
            histogram(error_norms, 50, 'Normalization', 'pdf'); hold on;
            
            xlabel('Position Error Magnitude [m]');
            ylabel('Density (PDF)');
            title(sprintf('Position Error Distribution: %s - %s', strrep(scenario_name, '_', ' '), filter_name));
            grid on;
            
            saveas(gcf, fullfile(plot_folder, sprintf('error_hist_%s_%s.png', scenario_name, filter_name)));
            close(gcf);
        end
    end
end