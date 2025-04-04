function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1 0 0;
                    0 1 0];
public_vars.kf.R = diag([0.0001  0.0001  0.1]);
public_vars.kf.Q = [0.2532  0  ;
                     0  0.2358 ];
if ~isfield(public_vars, 'mu')
public_vars.mu = [0,0,0];
end
public_vars.sigma = zeros(3);
public_vars.sigma(1:2,1:2) = cov(read_only_vars.gnss_history);
end

