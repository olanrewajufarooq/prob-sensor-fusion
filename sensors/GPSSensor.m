classdef GPSSensor < SensorModel
    % GPSSENSOR Measures position [px, py] from the 4-state vector [px, py, theta, v]'
    
    properties
        H
        noise_model
        name
    end
    
    methods
        function obj = GPSSensor(noise_model)
            obj.name = 'GPS';
            % H maps 4-state to 2D measurement
            obj.H = [1, 0, 0, 0;
                     0, 1, 0, 0];  
            obj.noise_model = noise_model;
        end
        
        function z = measure(obj, x)
            v = obj.noise_model.sample(1);
            z = obj.H * x + v;
        end
    end
end