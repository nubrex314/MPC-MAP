function [public_vars] = init_kalman_filter(read_only_vars, public_vars)
%INIT_KALMAN_FILTER Summary of this function goes here

public_vars.kf.C = [1 0 0;
                    0 1 0];
% public_vars.kf.R = diag([0.0001  0.0001  0.1]);
public_vars.kf.R= diag([0.00008  0.00008  0.00008]);
public_vars.kf.Q = [0.25  0  ;
                     0  0.25 ];
if ~isfield(public_vars, 'mu')
    if public_vars.kf_enabled
        public_vars.mu = public_vars.estimated_pose;
    else
        public_vars.mu =[1,1,1];
    end
end
public_vars.sigma = zeros(3);
public_vars.sigma(1:2,1:2) = cov(~any(isnan(read_only_vars.gnss_history), 2));
end

