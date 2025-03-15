function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
XR=read_only_vars.mocap_pose(1);
YR=read_only_vars.mocap_pose(2);
thetaR=read_only_vars.mocap_pose(3);
epsilon=0.2;
k=1;
%% I. Pick navigation target

%target = get_target(public_vars.estimated_pose, public_vars.path);
    if isempty(public_vars.path)
    disp('NO TRACK.');
    target=[XR,YR];
    else
    distance = norm([XR,YR]-public_vars.path(1,:));
        if distance < 0.4
            public_vars.path(1, :) = [];
        end
    target = public_vars.path(1,:);
    end

%% II. Compute motion vector

%public_vars.motion_vector = [0, 0];
xp=XR+epsilon*cos(thetaR);
yp=YR+epsilon*sin(thetaR);
dxp=k*(target(1)-xp);
dyp=k*(target(2)-yp);
%v=dxp*cos(thetaR)+dyp*sin(thetaR);
v=0.7;
u=(1/epsilon)*(-dxp*sin(thetaR)+dyp*cos(thetaR));
public_vars.motion_vector =kinematics(v,u);
end