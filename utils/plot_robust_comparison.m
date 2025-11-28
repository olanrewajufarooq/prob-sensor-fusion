function plot_robust_comparison(results_standard, results_robust, params)
    % Compare standard vs robust filter
    
    % Compute metrics for both
    mse_std = Metrics.computeMSE(mean(results_standard.errors.^2, 3));
    mse_rob = Metrics.computeMSE(mean(results_robust.errors.^2, 3));
    
    % NEES
    nees_std = [];
    nees_rob = [];
    for trial = 1:size(results_standard.errors, 3)
        nees_std = [nees_std; Metrics.computeNEES(results_standard.errors(:,:,trial), ...
            results_standard.P_history{trial})];
        nees_rob = [nees_rob; Metrics.computeNEES(results_robust.errors(:,:,trial), ...
            results_robust.P_history{trial})];
    end
    
    % Side-by-side comparison
    figure('Position', [100, 100, 1200, 500]);
    
    subplot(1, 2, 1);
    plot(mean(nees_std, 1), 'b-', 'LineWidth', 1.5); hold on;
    plot(mean(nees_rob, 1), 'r-', 'LineWidth', 1.5);
    yline(4, 'k--', 'Expected', 'LineWidth', 2);
    xlabel('Time step');
    ylabel('NEES');
    legend('Standard KF', 'Robust KF', 'Expected');
    title('NEES Comparison');
    grid on;
    
    subplot(1, 2, 2);
    data = [mse_std, mse_rob];
    bar(data);
    set(gca, 'XTickLabel', {'Standard', 'Robust'});
    ylabel('MSE');
    title('MSE Comparison');
    grid on;
    
    saveas(gcf, sprintf('plots/robust_comparison_%s.png', params.scenario));
    
    fprintf('\n=== Robust vs Standard Comparison ===\n');
    fprintf('Standard MSE: %.4f\n', mse_std);
    fprintf('Robust MSE: %.4f\n', mse_rob);
    fprintf('Standard Mean NEES: %.4f\n', mean(nees_std(:)));
    fprintf('Robust Mean NEES: %.4f\n', mean(nees_rob(:)));
    
    close all;
end