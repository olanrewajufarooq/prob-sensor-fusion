classdef (Abstract) SensorModel
    properties (Abstract)
        H           % Measurement matrix (nx by nz)
        noise_model % NoiseModel object
        name        % Sensor identifier
    end
    
    methods (Abstract)
        z = measure(obj, x)  % Generate measurement
    end
end