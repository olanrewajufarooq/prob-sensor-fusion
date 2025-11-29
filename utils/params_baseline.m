function params = params_baseline()
% PARAMS_BASELINE Baseline Parameters for Simulation (Non-linear Unicycle)

% Simulation Parameters
params.dt = 0.1;
params.T = 400; % 40 seconds of simulation
params.N_trials = 500; % Monte Carlo runs (to satisfy LLN)

% Motion Model Parameters (True Q for Unicycle Dynamics)
params.sigma_w_pos = 0.01; % Noise in position update components
params.sigma_w_vel = 0.05;
params.sigma_w_theta = 0.01;

% Diagonal Q matrix for Additive Noise in [px, py, theta, v]
params.Q = diag([
    params.sigma_w_pos^2, ...
    params.sigma_w_pos^2, ...
    params.sigma_w_theta^2, ...
    params.sigma_w_vel^2
]);

% Sensor Model Parameters (Used for standard R)
params.sigma_gps = 2.0; % Position standard deviation
params.sigma_odom = 0.3; % Velocity/Heading standard deviation

% Filter Initial Conditions
params.x0 = [0; 0; 0; 0]; % Start at origin, 0 heading, 0 velocity
params.P0 = diag([1.0, 1.0, 0.5, 0.5]);

% PID Controller Parameters (Example gains for a smoother circle)
% Kp for [dist_err; heading_err]
params.pid_params.Kp = [0.8; 1.5];
params.pid_params.Ki = [0.0; 0.0];
params.pid_params.Kd = [0.1; 0.2];

% Scenario name
params.scenario = 'baseline';

end