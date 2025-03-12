function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here
m=length(public_vars.particles);
estimated_pose = [mean(public_vars.particles(1:m*0.7,1)),mean(public_vars.particles(1:m*0.7,2)),mean(public_vars.particles(1:m*0.7,3))];

end

