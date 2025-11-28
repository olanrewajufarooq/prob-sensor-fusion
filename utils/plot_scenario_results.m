function plot_scenario_results(results, params, scenario_name)
    % Generate standard plots for a single scenario
    
    % Compute metrics
    errors = results.errors;
    mse = Metrics.computeMSE(mean(errors.^2, 3));
    
    % Extract single trial for visualization
    trial_errors = errors(:, :, 1);
    trial_P = results.P_history{1};
    
    % Compute NEES for all trials
    nees_all = [];
    for trial = 1:size(errors, 3)
        trial_err = errors(:, :, trial);
        trial_nees = Metrics.computeNEES(trial_err, results.P_history{trial});
        nees_all = [nees_all; trial_nees];
    end
    nees_mean = mean(nees_all, 1);
    
    % Plot 1: Trajectory with confidence ellipses
    figure('Position', [100, 100, 800, 600]);
    
    % Generate true trajectory
    dynamics = RobotDynamics(params.dt, params.sigma_w);
    x_true = zeros(4, params.T);
    x_true(:, 1) = params.x0;
    for t = 2:params.T
        x_true(:, t) = dynamics.F * x_true(:, t-1);
    end
    
    x_est = x_true - trial_errors;
    Plotter.plotTrajectory(x_true, x_est, trial_P);
    title(sprintf('Trajectory - %s', strrep(scenario_name, '_', ' ')));
    saveas(gcf, sprintf('plots/trajectory_%s.png', scenario_name));
    
    % Plot 2: NEES time series
    figure('Position', [100, 100, 800, 600]);
    Plotter.plotNEES(nees_mean, 4);
    title(sprintf('NEES - %s (Mean=%.2f)', strrep(scenario_name, '_', ' '), mean(nees_mean)));
    saveas(gcf, sprintf('plots/nees_%s.png', scenario_name));
    
    % Plot 3: Error histogram
    figure('Position', [100, 100, 800, 600]);
    Plotter.plotErrorHistogram(errors(:, :));
    title(sprintf('Error Distribution - %s', strrep(scenario_name, '_', ' ')));
    saveas(gcf, sprintf('plots/error_hist_%s.png', scenario_name));
    
    % Print summary statistics
    fprintf('\n=== Results for %s ===\n', scenario_name);
    fprintf('MSE: %.4f\n', mse);
    fprintf('Mean NEES: %.4f\n', mean(nees_mean));
    fprintf('RMSE: %.4f m\n', sqrt(mse));
    
    close all;
end