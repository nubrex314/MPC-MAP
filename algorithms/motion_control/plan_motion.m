function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here

% I. Pick navigation target

target = get_target(public_vars.estimated_pose, public_vars.path);


% II. Compute motion vector

public_vars.motion_vector = [0, 0];
count=read_only_vars.counter;
% switch count
%     case num2cell(1:50)
%     public_vars.motion_vector = [1, 1];
%     case num2cell(51:110)
%     public_vars.motion_vector = [0.9 , 1];
%     case num2cell(111:139)
%     public_vars.motion_vector = [1, 1];
%     case num2cell(140:195)
%     public_vars.motion_vector = [1 , 0.9];
%     case num2cell(196:300)
%     public_vars.motion_vector = [1, 1];
% end


end