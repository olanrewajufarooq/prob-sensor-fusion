function params = params_baseline()
%PARAMS_BASELINE Baseline Parameters for Simulation

% Simulation Parameters
params.dt = 0.1;
params.T = 200;
params.N_trials = 500;

% Motion Model Parameters
params.sigma_w = 0.1;

% Sensor Model Parameters
params.sigma_gps = 2.0;
params.sigma_odom = 0.3;

% Initial Conditions
params.x0 = [0; 0; 1; 0.5];
params.P0 = diag([1.0, 1.0, 0.5, 0.5]);

% Scenario name
params.scenario = 'baseline';

end

