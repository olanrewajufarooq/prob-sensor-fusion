function params= params_correlated(rho)
% PARAMS_CORRELATED Parameters for the correlated noise case.
%   rho: degree of correlation (e.g., 0.7)

params = params_baseline();
params.rho = rho;
params.scenario = sprintf('correlated_rho%.1f', rho);

end