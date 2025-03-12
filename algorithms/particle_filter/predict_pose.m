function [new_pose] = predict_pose(old_pose, motion_vector, read_only_vars)

    v = sum(motion_vector) / 2;
    w = (motion_vector(1) - motion_vector(2)) / read_only_vars.agent_drive.interwheel_dist;
    d_t = read_only_vars.sampling_period; 

    x_new = old_pose(1) + v*cos(old_pose(3))*d_t;
    y_new = old_pose(2) + v*sin(old_pose(3))* d_t;
    theta_new = old_pose(3) + w*d_t;
    x_new = x_new+((randn(1)-0.5)/10);
    y_new = y_new+((randn(1)-0.5)/10);
    theta_new = theta_new + ((randn(1)-0.5)/10);
    new_pose = [x_new, y_new, theta_new];
end