classdef GaussianNoise < NoiseModel
    % GAUSSIANNOISE Standard Gaussian noise model.
    
    properties
        R       % Covariance
        dim     % Noise dimension
    end
    
    methods
        function obj = GaussianNoise(R)
            obj.R = R;
            obj.dim = size(R, 1);
        end
        
        function v = sample(obj, n)
            if nargin < 2
                n = 1;
            end
            % mvnrnd returns n x dim, transpose to dim x n
            v = mvnrnd(zeros(obj.dim, 1), obj.R, n)';
        end
        
        function R_true = getTrueCovariance(obj)
            R_true = obj.R;
        end
    end
end