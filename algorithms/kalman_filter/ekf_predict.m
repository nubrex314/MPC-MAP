function [new_mu, new_sigma] = ekf_predict(mu, sigma, u, kf, sampling_period)
%EKF_PREDICT Summary of this function goes here
    v = sum(u) / 2;
    w = (u(1) - u(2)) / 0.2;
    d_t = sampling_period; 
    x_new = mu(1) + v*cos(mu(3))*d_t;
    y_new = mu(2) + v*sin(mu(3))* d_t;
    theta_new = mu(3) + w*d_t;

new_mu = [x_new; y_new ; theta_new];

G=[1 0 -sin(mu(3))*v*d_t;
   0 1 cos(mu(3))*v*d_t
   0 0       1          ];

new_sigma = G*sigma*G'+kf.R;

end

