function [weights] = weight_particles(particle_measurements, lidar_distances)
%WEIGHT_PARTICLES Summary of this function goes here

N = size(particle_measurements, 1);

weights=1 ./ sqrt(sum((particle_measurements-lidar_distances).^2,2));

weights(isnan(weights)) = 0.000001;

 weights=weights/sum(weights);
end

