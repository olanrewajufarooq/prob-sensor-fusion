classdef (Abstract) NoiseModel
    %NOISEMODEL Abstract base class for measurement noise models
    
    properties (Abstract)
        R       % Covariance
        dim     % Noise dimension
    end
    
    methods (Abstract)
        v = sample(obj, n)  % Sample n noise vectors
        R_true = getTrueCovariance(obj)  % Return actual covariance
    end
end

