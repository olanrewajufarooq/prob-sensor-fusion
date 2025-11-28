classdef GPSSensor < SensorModel
    properties
        H
        noise_model
        name
    end
    
    methods
        function obj = GPSSensor(noise_model)
            obj.name = 'GPS';
            obj.H = [1, 0, 0, 0;
                     0, 1, 0, 0];  % Measures position only
            obj.noise_model = noise_model;
        end
        
        function z = measure(obj, x)
            v = obj.noise_model.sample(1);
            z = obj.H * x + v;
        end
    end
end