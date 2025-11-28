function params= params_correlated(rho)
%PARAMS_CORRELATED Parameters for the correlated case
%   rho:    degree of correlation

params = params_baseline();
params.rho = rho;
params.scenario = sprintf('correlated_rho%.1f', rho);

end

