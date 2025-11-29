classdef Metrics
    % METRICS Calculates key estimation metrics: MSE, NEES, and Tail Probability.
    
    methods (Static)
        function mse = computeMSE(errors)
            % errors: nx x T x N_trials matrix
            % Returns scalar Mean Squared Error averaged over time, trials, and state.
            
            % Compute squared norm of error at each time step/trial
            squared_errors = sum(errors.^2, 1); % Sum over state dimension
            
            % Average over time and trials
            mse = mean(squared_errors(:)); 
        end
        
        function nees = computeNEES(errors, P_history)
            % errors: nx x T matrix (single trial)
            % P_history: cell array of covariances {P1, P2, ..., PT}
            T = size(errors, 2);
            nees = zeros(1, T);
            
            for t = 1:T
                e = errors(:, t);
                P = P_history{t};
                
                % NEES = e' * P^-1 * e
                nees(t) = e' / P * e;
            end
        end
        
        function prob = tailProbability(errors, threshold)
            % errors: nx x T x N_trials matrix
            % Fraction of error magnitudes exceeding threshold
            
            % Reshape errors to (nx x T*N_trials)
            errors_flat = reshape(errors, size(errors, 1), []);
            
            % Compute 2-norm (magnitude) of the error vector
            norms = sqrt(sum(errors_flat.^2, 1));
            
            % Calculate fraction exceeding threshold
            prob = mean(norms > threshold);
        end
    end
end