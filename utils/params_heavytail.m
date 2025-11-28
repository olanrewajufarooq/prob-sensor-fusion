function params = params_heavytail(pi_outlier, lambda)
%PARAMS_HEAVYTAIL Parameters for the heavy-tail case
%   Model:  v ~ (1-π)N(0, R) + π×N(0, λ²R)
%
%   pi_outlier:     probability of outliers
%   lambda:         std multiplier for outliers

params = params_baseline();
params.pi_outlier = pi_outlier;
params.lambda = lambda;
params.scenario = sprintf('heavytail_pi%.2f_lambda%d', pi_outlier, lambda);

end

