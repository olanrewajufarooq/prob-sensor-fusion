classdef CorrelatedGaussianNoise < NoiseModel
    % CORRELATEDGAUSSIANNOISE Noise model for intentionally mis-specified correlation.
    
    properties
        R           % True correlated covariance
        R_assumed   % Diagonal covariance assumed by filter
        dim
        rho         % Correlation coefficient
    end
    
    methods
        function obj = CorrelatedGaussianNoise(R_true, R_assumed)
            % R_true: The actual, non-diagonal covariance
            % R_assumed: The diagonal covariance the filter will use
            obj.R = R_true;
            obj.R_assumed = R_assumed;
            obj.dim = size(R_true, 1);
            
            % If R_true is 2x2, we can extract rho for informational purposes.
            if obj.dim == 2
                sigma1 = sqrt(R_true(1,1));
                sigma2 = sqrt(R_true(2,2));
                obj.rho = R_true(1,2) / (sigma1 * sigma2);
            else
                obj.rho = NaN;
            end
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