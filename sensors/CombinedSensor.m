classdef CombinedSensor < SensorModel
    properties
        H
        noise_model
        name
    end
    
    methods
        function obj = CombinedSensor(noise_model)
            obj.name = 'Combined';
            obj.H = eye(4);  % Measures full state
            obj.noise_model = noise_model;
        end
        
        function z = measure(obj, x)
            v = obj.noise_model.sample(1);
            z = obj.H * x + v;
        end
    end
end