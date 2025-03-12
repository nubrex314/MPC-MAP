function [measurement] = compute_lidar_measurement(map, pose, lidar_config)
%COMPUTE_MEASUREMENTS Summary of this function goes here

measurement = zeros(1, length(lidar_config));
for i = 1:length(measurement)
        intersections = ray_cast(pose(1:2),map.walls , pose(3)+lidar_config(i));

        dis = sqrt((intersections(:,1) - pose(1)).^2 + (intersections(:,2) - pose(2)).^2);
        distance = min(dis);
        measurement(i) = distance;
end
end
