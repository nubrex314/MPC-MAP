function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)
%PREDICT_POSE Summary of this function goes here
    motion_vector_pred=motion_vector + ((randn(1,2)-0.5)/10);
    theta = old_pose(3) ;
    dS = sum(motion_vector) / 2;
    dTheta = (motion_vector_pred(1) - motion_vector_pred(2))/read_only_vars.agent_drive.interwheel_dist * read_only_vars.sampling_period;
    dX = dS * cos(theta)* read_only_vars.sampling_period;
    dY = dS * sin(theta)* read_only_vars.sampling_period;
    
    X_new = old_pose(1) + dX;
    Y_new = old_pose(2) + dY;

new_pose = [X_new, Y_new, dTheta];
end

