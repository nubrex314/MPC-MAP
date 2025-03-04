function [target] = get_target(estimated_pose, path)
%GET_TARGET Summary of this function goes here
distance = norm(estimated_pose(1:2)-path(1,:));
if distance < 0.5
    path(1, :) = [];
end

target = path(1,:);

end

