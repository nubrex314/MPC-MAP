function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here
nan_mask=isinf(lidar_distances);
valid_row = all(isnan(particle_measurements) == nan_mask, 2);
weights(~valid_row,:) = 1e-9;
particle_measurements = particle_measurements( valid_row,:);
particle_measurements = particle_measurements(: ,~nan_mask);
lidar_distances = lidar_distances(~nan_mask);
weights(valid_row)=1 ./ sqrt(sum((particle_measurements-lidar_distances).^2,2));
%weights(isnan(weights)) = 0.000000001;
 weights=weights/sum(weights);
% sigma=0.5;
%  weights = exp(-0.5 * ((lidar_distances-particle_measurements) ./ sigma) .^ 2);
% weights(weights == 0) = 1;
% weights = prod(weights, 2);
% weights(isnan(weights)) = 1/10^200;


end

