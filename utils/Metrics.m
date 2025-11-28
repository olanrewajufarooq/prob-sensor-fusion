classdef Metrics
    methods (Static)
        function mse = computeMSE(errors)
            % errors: nx x T matrix
            mse = mean(sum(errors.^2, 1));
        end
        
        function nees = computeNEES(errors, P_history)
            % errors: nx x T, P_history: cell array of covariances
            T = size(errors, 2);
            nees = zeros(1, T);
            
            for t = 1:T
                e = errors(:, t);
                P = P_history{t};
                nees(t) = e' / P * e;
            end
        end
        
        function prob = tailProbability(errors, threshold)
            % Fraction of errors exceeding threshold
            norms = sqrt(sum(errors.^2, 1));
            prob = mean(norms > threshold);
        end
    end
end
