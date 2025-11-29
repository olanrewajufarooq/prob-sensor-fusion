classdef MixtureNoise < NoiseModel
    % MIXURENOISE Models heavy-tailed noise as a mixture of two Gaussians.
    % Used for the non-Gaussian mis-specification scenario.
    
    properties
        R           % Nominal covariance
        R_outlier   % Outlier covariance (lambda^2 * R)
        dim
        pi_outlier  % Outlier probability (epsilon)
        lambda      % Outlier scale factor
    end
    
    methods
        function obj = MixtureNoise(R, pi_outlier, lambda)
            obj.R = R;
            obj.dim = size(R, 1);
            obj.pi_outlier = pi_outlier;
            obj.lambda = lambda;
            obj.R_outlier = lambda^2 * R;
        end
        
        function v = sample(obj, n)
            if nargin < 2, n = 1; end
            v = zeros(obj.dim, n);
            
            for i = 1:n
                if rand() < obj.pi_outlier
                    % Sample from outlier distribution
                    v(:,i) = mvnrnd(zeros(obj.dim,1), obj.R_outlier)';
                else
                    % Sample from nominal distribution
                    v(:,i) = mvnrnd(zeros(obj.dim,1), obj.R)';
                end
            end
        end
        
        function R_true = getTrueCovariance(obj)
            % True average covariance: E[v v^T]
            R_true = (1-obj.pi_outlier)*obj.R + obj.pi_outlier*obj.R_outlier;
        end
    end
end