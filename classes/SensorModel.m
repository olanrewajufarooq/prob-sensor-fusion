classdef (Abstract) SensorModel
    properties (Abstract)
        H           % Measurement matrix
        noise_model % NoiseModel object
        name        % Sensor identifier
    end
    
    methods (Abstract)
        z = measure(obj, x)  % Generate measurement
    end
end