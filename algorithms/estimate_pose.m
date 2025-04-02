function [estimated_pose] = estimate_pose(public_vars)
%ESTIMATE_POSE Summary of this function goes here
if public_vars.kf_enabled && ~public_vars.pf_enabled
    estimated_pose = public_vars.mu';
elseif public_vars.pf_enabled && ~public_vars.kf_enabled
    perc=public_vars.resablicng_per;
    m=length(public_vars.particles);
    estimated_pose = [median(public_vars.particles(1:m*perc,1)),median(public_vars.particles(1:m*perc,2)),median(public_vars.particles(1:m*perc,3))];
% elseif public_vars.kf_enabled && public_vars.pf_enabled
%     perc=public_vars.resablicng_per;
%     m=length(public_vars.particles);
%     estimated_pose = [mean(public_vars.particles(1:m*perc,1)),mean(public_vars.particles(1:m*perc,2)),mean(public_vars.particles(1:m*perc,3))];
%     if norm(estimated_pose-public_vars.estimated_pose)>1
%         estimated_pose = predict_pose(public_vars.estimated_pose,public_vars.motion_vector);
%     end
else
    estimated_pose = nan(1,3);
end
end