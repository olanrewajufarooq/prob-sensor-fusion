classdef GaussianNoise < NoiseModel
    %GAUSSIANNOISE Summary of this class goes here
    
    properties
        R       % Covariance
        dim     % Noise dimension
    end
    
    methods
        function obj = GaussianNoise(R)
            %GAUSSIANNOISE Construct an instance of this class
            %   R:  Covariance
            obj.R = R;
            obj.dim = size(R, 1);
        end
        
        function v = sample(obj, n)
            %SAMPLE Summary of this method goes here
            %   n:  Vector size of the output
            
            if nargin < 2
                n = 1;
            end

            v = mvnrnd(zeros(obj.dim, 1), obj.R, n)';
        end

        function R_true = getTrueCovariance(obj)
            R_true = obj.R;
        end
    end
end

