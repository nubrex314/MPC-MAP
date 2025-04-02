function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here

if public_vars.change~=0
        estimated_pose = predict_pose(public_vars.estimated_pose,public_vars.motion_vector);
elseif public_vars.kf_enabled 
    estimated_pose = public_vars.mu';
elseif public_vars.pf_enabled
    perc=public_vars.resablicng_per;
    m=length(public_vars.particles);
    estimated_pose = [median(public_vars.particles(1:m*perc,1)),median(public_vars.particles(1:m*perc,2)),median(public_vars.particles(1:m*perc,3))];
else
    estimated_pose = nan(1,3);
end
end