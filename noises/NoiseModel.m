classdef (Abstract) NoiseModel
    % NOISEMODEL Abstract base class for process or measurement noise models
    
    properties (Abstract)
        R       % Covariance (nominal or true)
        dim     % Noise dimension
    end
    
    methods (Abstract)
        v = sample(obj, n)          % Sample n noise vectors
        R_true = getTrueCovariance(obj) % Return actual covariance
    end
end