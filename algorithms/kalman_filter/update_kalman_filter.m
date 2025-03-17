function [mu, sigma] = update_kalman_filter(read_only_vars, public_vars)
%UPDATE_KALMAN_FILTER Summary of this function goes here
if read_only_vars.counter<20
  public_vars.mu = [mean(read_only_vars.gnss_history(:,1)) mean(read_only_vars.gnss_history(:,2)) public_vars.mu(3)];
else
  public_vars.kf.R= diag([0.0001  0.0001  0.00001]);
end
mu = public_vars.mu;
sigma = public_vars.sigma;

% I. Prediction
u = public_vars.motion_vector;
[mu, sigma] = ekf_predict(mu, sigma, u, public_vars.kf, read_only_vars.sampling_period);

% II. Measurement
z = read_only_vars.gnss_position';
[mu, sigma] = kf_measure(mu, sigma, z, public_vars.kf);

end

