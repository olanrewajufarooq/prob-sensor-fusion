function sensors = create_correlated_sensors(sigma_gps, sigma_odom, rho)
    % Creates a pair of sensors with correlated measurements
    % This is a simplified version - you may need to customize
    
    % Create a joint noise model that generates correlated samples
    % For this, we need a custom sensor that measures both position and velocity
    % with correlation
    
    % Combined measurement: [px; py; vx; vy]
    H_combined = eye(4);
    
    % Correlated noise covariance
    R_combined = [
        sigma_gps^2, 0, rho*sigma_gps*sigma_odom, 0;
        0, sigma_gps^2, 0, rho*sigma_gps*sigma_odom;
        rho*sigma_gps*sigma_odom, 0, sigma_odom^2, 0;
        0, rho*sigma_gps*sigma_odom, 0, sigma_odom^2
    ];
    
    % Create custom sensor
    combined_noise = GaussianNoise(R_combined);
    sensor = CombinedSensor(combined_noise);
    
    sensors = {sensor};
end