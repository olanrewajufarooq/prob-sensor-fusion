classdef Plotter
    methods (Static)
        function plotTrajectory(x_true, x_est, P_history)
            figure;
            plot(x_true(1,:), x_true(2,:), 'k-', 'LineWidth', 2); hold on;
            plot(x_est(1,:), x_est(2,:), 'r--', 'LineWidth', 1.5);
            
            % Plot covariance ellipses every 10 steps
            for t = 1:10:length(P_history)
                Plotter.plotCovarianceEllipse(x_est(1:2,t), P_history{t}(1:2,1:2), 2);
            end
            
            xlabel('x position [m]');
            ylabel('y position [m]');
            legend('True', 'Estimated', '2\sigma ellipse');
            title('Robot Trajectory with Uncertainty');
            grid on; axis equal;
        end
        
        function plotCovarianceEllipse(mu, Sigma, nsigma)
            [V, D] = eig(Sigma);
            theta = linspace(0, 2*pi, 100);
            ellipse = nsigma * V * sqrt(D) * [cos(theta); sin(theta)];
            plot(mu(1) + ellipse(1,:), mu(2) + ellipse(2,:), 'b-', 'LineWidth', 0.5);
        end
        
        function plotNEES(nees, nx)
            figure;
            plot(nees, 'LineWidth', 1.5); hold on;
            yline(nx, 'r--', 'Expected NEES', 'LineWidth', 2);
            xlabel('Time step');
            ylabel('NEES');
            title('Normalized Estimation Error Squared');
            grid on;
        end
        
        function plotErrorHistogram(errors, theoretical_std)
            figure;
            error_norms = sqrt(sum(errors.^2, 1));
            histogram(error_norms, 50, 'Normalization', 'pdf'); hold on;
            
            % Overlay theoretical distribution if available
            if nargin > 1
                x = linspace(0, max(error_norms), 100);
                plot(x, chi2pdf(x.^2/theoretical_std^2, size(errors,1)), 'r-', 'LineWidth', 2);
            end
            
            xlabel('Error magnitude');
            ylabel('Density');
            title('Error Distribution');
            grid on;
        end
    end
end