function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 1;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    if isempty(path)
        return
    end
    path = [path(:,2) ./ read_only_vars.discrete_map.dims(2).*read_only_vars.map.limits(4)...
            path(:,1) ./ read_only_vars.discrete_map.dims(1).*read_only_vars.map.limits(3)-0.2];

    path = smooth_path(path);
else
    %path = indor_1_path;
   % path = outdor_1_path;
    path = outdor_1_path_v2;
    
end

end

