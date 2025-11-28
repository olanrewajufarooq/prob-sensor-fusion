classdef CorrelatedGaussianNoise < NoiseModel
    properties
        R           % True correlated covariance
        R_assumed   % Diagonal covariance assumed by filter
        dim
        rho         % Correlation coefficient
    end
    
    methods
        function obj = CorrelatedGaussianNoise(sigma1, sigma2, rho)
            obj.dim = 2;
            obj.rho = rho;
            
            % True covariance with correlation
            obj.R = [
                sigma1^2, rho*sigma1*sigma2;
                rho*sigma1*sigma2, sigma2^2
            ];
            
            % Filter assumes diagonal
            obj.R_assumed = diag([sigma1^2, sigma2^2]);
        end
        
        function v = sample(obj, n)
            if nargin < 2
                n = 1; 
            end
            v = mvnrnd(zeros(obj.dim, 1), obj.R, n)';
        end
        
        function R_true = getTrueCovariance(obj)
            R_true = obj.R;
        end
        
        function R_filt = getAssumedCovariance(obj)
            R_filt = obj.R_assumed;
        end
    end
end