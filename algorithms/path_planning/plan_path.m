function [path] = plan_path(read_only_vars, public_vars)
%PLAN_PATH Summary of this function goes here

planning_required = 0;

if planning_required
    
    path = astar(read_only_vars, public_vars);
    
    path = smooth_path(path);
    
else
    %path = indor_1_path;
   % path = outdor_1_path;
    path = outdor_1_path_v2;
    
end

end

