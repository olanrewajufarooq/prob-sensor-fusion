classdef MixtureNoise < NoiseModel
    properties
        R           % Nominal covariance
        R_outlier   % Outlier covariance (lambda * R)
        dim
        pi_outlier  % Outlier probability
        lambda      % Outlier scale factor
    end
    
    methods
        function obj = MixtureNoise(R, pi_outlier, lambda)
            obj.R = R;
            obj.dim = size(R, 1);
            obj.pi_outlier = pi_outlier;
            obj.lambda = lambda;
            obj.R_outlier = lambda * R;
        end
        
        function v = sample(obj, n)
            if nargin < 2, n = 1; end
            v = zeros(obj.dim, n);
            
            for i = 1:n
                if rand() < obj.pi_outlier
                    v(:,i) = mvnrnd(zeros(obj.dim,1), obj.R_outlier)';
                else
                    v(:,i) = mvnrnd(zeros(obj.dim,1), obj.R)';
                end
            end
        end
        
        function R_true = getTrueCovariance(obj)
            R_true = (1-obj.pi_outlier)*obj.R + obj.pi_outlier*obj.R_outlier;
        end
    end
end
