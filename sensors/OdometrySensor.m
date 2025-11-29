classdef OdometrySensor < SensorModel
    % ODOMETRYSENSOR Measures velocity and heading [v, theta] from the 4-state vector.
    
    properties
        H
        noise_model
        name
    end
    
    methods
        function obj = OdometrySensor(noise_model)
            obj.name = 'Odometry';
            % H measures [theta, v] from state [px, py, theta, v]'
            obj.H = [0, 0, 1, 0;
                     0, 0, 0, 1];
            obj.noise_model = noise_model;
        end
        
        function z = measure(obj, x)
            v = obj.noise_model.sample(1);
            z = obj.H * x + v;
        end
    end
end