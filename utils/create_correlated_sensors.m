function sensors = create_correlated_sensors(sigma_gps, sigma_odom, rho)
    % Creates a single CombinedSensor that measures the full state [px, py, theta, v] 
    % with non-diagonal correlated Gaussian noise R.
    
    % We assume correlation occurs between [px, py] and [theta, v] measurements,
    % or between the two measured pairs (e.g., px and theta).
    % For simplicity and impact, we will correlate the two main measurement pairs: 
    % Position (px, py) measurement noise and Velocity (v) measurement noise.
    
    % The simplest way to apply this is to assume correlation between the 
    % x-position noise (v_px) and the velocity noise (v_v).
    
    % State: [px, py, theta, v]' (Dimension 4)
    % True Covariance R_true (4x4)
    
    sigma_px = sigma_gps;
    sigma_py = sigma_gps;
    sigma_theta = 0.1; % Assume small, un-correlated theta noise
    sigma_v = sigma_odom;
    
    % True Covariance with correlation between v_px and v_v
    R_true = [
        sigma_px^2, 0, 0, rho*sigma_px*sigma_v; % Correlate px and v noise
        0, sigma_py^2, 0, 0;
        0, 0, sigma_theta^2, 0;
        rho*sigma_px*sigma_v, 0, 0, sigma_v^2
    ];
    
    % Filter Assumed Covariance (Diagonal R)
    R_assumed = diag([sigma_px^2, sigma_py^2, sigma_theta^2, sigma_v^2]);
    
    % Create combined sensor and noise model
    noise_model = CorrelatedGaussianNoise(R_true, R_assumed);
    sensor = CombinedSensor(noise_model); % Measures full state H=I
    
    sensors = {sensor};
end