function [particles] = update_particle_filter(read_only_vars, public_vars)
%UPDATE_PARTICLE_FILTER Summary of this function goes here
particles = public_vars.particles;
% I. Prediction
for i=1:size(particles, 1)
    particles(i,:) = predict_pose(particles(i,:), public_vars.motion_vector, read_only_vars);
end

% II. Correction
measurements = zeros(size(particles,1), length(read_only_vars.lidar_config));
for i=1:size(particles, 1)
    measurements(i,:) = compute_lidar_measurement(read_only_vars.map, particles(i,:), read_only_vars.lidar_config);
end
weights = weight_particles(measurements, read_only_vars.lidar_distances);

% III. Resampling
particles = resample_particles(particles, weights);

m=length(particles);
percen=0.9;
for i=m*percen:m
    [~,particles(i,:)] = init_particle_filter(read_only_vars, public_vars, 1);
end
end

