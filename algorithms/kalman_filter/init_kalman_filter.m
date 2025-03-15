function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [];
public_vars.kf.R = [];
public_vars.kf.Q = [];

public_vars.mu = [0.5032;0.4856];
public_vars.sigma = [0.2532 , 0.0103 ;...
                     0.0103 , 0.2358];

end

