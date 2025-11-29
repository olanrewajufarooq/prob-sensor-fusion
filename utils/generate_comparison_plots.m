function generate_comparison_plots(all_results, plot_folder)
    % GENERATE_COMPARISON_PLOTS Generates comprehensive bar charts and time series plots 
    % comparing KF, RKF, EKF, and RobustEKF for all scenarios.
    
    if ~exist(plot_folder, 'dir')
        mkdir(plot_folder);
    end
    
    scenarios = fieldnames(all_results);
    filter_labels = {'KF', 'RobustKF', 'EKF', 'RobustEKF'};
    nx = 4; % State dimension

    for i = 1:length(scenarios)
        scenario_name = scenarios{i};
        data = all_results.(scenario_name);
        
        % Initialize metrics storage for this scenario (4 filters)
        mse_values = zeros(1, 4);
        mean_nees_values = zeros(1, 4);
        nees_time_series = zeros(4, size(data.KF.errors, 2)); % 4 filters x T steps
        
        for f = 1:4
            filter_name = filter_labels{f};
            results = data.(filter_name);
            T = size(results.errors, 2);
            
            % 1. MSE
            mse_values(f) = Metrics.computeMSE(results.errors);
            
            % 2. NEES Time Series
            nees_all = zeros(size(results.errors, 3), T);
            for trial = 1:size(results.errors, 3)
                nees_all(trial, :) = Metrics.computeNEES(results.errors(:,:,trial), results.P_history{trial});
            end
            mean_nees_values(f) = mean(nees_all(:));
            nees_time_series(f, 1:T) = mean(nees_all, 1);
        end
        
        % --- Plot 1: Bar Chart Comparison (MSE and NEES) ---
        figure('Position', [100, 100, 1200, 500]);
        
        % MSE Plot
        subplot(1, 2, 1);
        bar(mse_values);
        set(gca, 'XTickLabel', filter_labels, 'XTickLabelRotation', 0);
        ylabel('Mean Squared Error (MSE)');
        title(sprintf('MSE Comparison - %s', strrep(scenario_name, '_', ' ')));
        grid on;
        
        % NEES Plot
        subplot(1, 2, 2);
        bar(mean_nees_values); hold on;
        yline(nx, 'r--', 'Expected NEES=4', 'LineWidth', 2);
        set(gca, 'XTickLabel', filter_labels, 'XTickLabelRotation', 0);
        ylabel('Mean NEES');
        title(sprintf('Mean NEES Comparison - %s', strrep(scenario_name, '_', ' ')));
        grid on;
        
        saveas(gcf, fullfile(plot_folder, sprintf('bar_comparison_%s.png', scenario_name)));
        close(gcf);
        
        % --- Plot 2: NEES Time Series ---
        figure('Position', [100, 100, 800, 600]);
        plot(nees_time_series', 'LineWidth', 1.5);
        yline(nx, 'k--', 'Expected NEES', 'LineWidth', 2);
        xlabel('Time step');
        ylabel('Mean NEES');
        title(sprintf('NEES Time Series - %s', strrep(scenario_name, '_', ' ')));
        legend(filter_labels, 'Location', 'best');
        grid on;
        
        saveas(gcf, fullfile(plot_folder, sprintf('nees_timeseries_%s.png', scenario_name)));
        close(gcf);
    end
    
    fprintf('\nGenerated all comprehensive comparison plots in %s!\n', plot_folder);
end