function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
% weights=1 ./ sqrt(sum((particle_measurements-lidar_distances).^2,2));
% weights(isnan(weights)) = 0.000001;
sigma=0.5;
 weights = exp(-0.5 * ((lidar_distances-particle_measurements) ./ sigma) .^ 2);
weights(weights == 0) = 1;
weights = prod(weights, 2);
weights(isnan(weights)) = 1/10^200;

 % weights=weights/sum(weights);
end

