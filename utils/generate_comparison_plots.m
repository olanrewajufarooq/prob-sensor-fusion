function generate_comparison_plots(all_results)
    % Generate comprehensive comparison across all scenarios
    
    scenarios = fieldnames(all_results);
    n_scenarios = length(scenarios);
    
    % Extract MSE and NEES for each scenario
    mse_values = zeros(1, n_scenarios);
    nees_values = zeros(1, n_scenarios);
    scenario_labels = cell(1, n_scenarios);
    
    for i = 1:n_scenarios
        scenario_name = scenarios{i};
        data = all_results.(scenario_name);
        
        if isfield(data, 'results')
            results = data.results;
        elseif isfield(data, 'results_robust')
            results = data.results_robust;
        else
            continue;
        end
        
        mse_values(i) = Metrics.computeMSE(mean(results.errors.^2, 3));
        
        % Compute mean NEES
        nees_all = [];
        for trial = 1:size(results.errors, 3)
            nees_trial = Metrics.computeNEES(results.errors(:,:,trial), results.P_history{trial});
            nees_all = [nees_all; nees_trial];
        end
        nees_values(i) = mean(nees_all(:));
        
        scenario_labels{i} = strrep(scenario_name, '_', ' ');
    end
    
    % Plot 1: MSE comparison bar chart
    figure('Position', [100, 100, 1200, 600]);
    bar(mse_values);
    set(gca, 'XTickLabel', scenario_labels, 'XTickLabelRotation', 45);
    ylabel('MSE');
    title('MSE Comparison Across All Scenarios');
    grid on;
    saveas(gcf, 'plots/mse_comparison_all.png');
    
    % Plot 2: NEES comparison
    figure('Position', [100, 100, 1200, 600]);
    bar(nees_values); hold on;
    yline(4, 'r--', 'Expected NEES=4', 'LineWidth', 2);
    set(gca, 'XTickLabel', scenario_labels, 'XTickLabelRotation', 45);
    ylabel('Mean NEES');
    title('NEES Comparison Across All Scenarios');
    grid on;
    saveas(gcf, 'plots/nees_comparison_all.png');
    
    % Plot 3: MSE vs Correlation (if correlated scenarios exist)
    corr_idx = contains(scenarios, 'correlated');
    if any(corr_idx)
        figure('Position', [100, 100, 800, 600]);
        rho_vals = [0.3, 0.5, 0.7, 0.9];
        corr_mse = mse_values(corr_idx);
        plot(rho_vals, corr_mse(1:length(rho_vals)), 'bo-', 'LineWidth', 2, 'MarkerSize', 10);
        xlabel('Correlation coefficient \rho');
        ylabel('MSE');
        title('MSE vs Correlation');
        grid on;
        saveas(gcf, 'plots/mse_vs_rho.png');
    end
    
    close all;
    
    fprintf('\nGenerated all comparison plots!\n');
end