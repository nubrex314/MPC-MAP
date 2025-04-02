function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
XR=public_vars.estimated_pose(1);
YR=public_vars.estimated_pose(2);
thetaR=public_vars.estimated_pose(3);
epsilon=0.3;
k=1;
%% I. Pick navigation target

%target = get_target(public_vars.estimated_pose, public_vars.path);
    if isempty(public_vars.path)
    disp('NO TRACK.');
    target=[XR,YR];
    else
    distance = norm([XR,YR]-public_vars.path(1,:));
        if distance < 0.5
            if(size(public_vars.path,1)~=1)
            public_vars.path(1, :) = [];
            else
            public_vars.lost=1;
            end
        end
    target = public_vars.path(1,:);
    end

%% II. Compute motion vector

%public_vars.motion_vector = [0, 0];
xp=XR+epsilon*cos(thetaR);
yp=YR+epsilon*sin(thetaR);
dxp=k*(target(1)-xp);
dyp=k*(target(2)-yp);
v=dxp*cos(thetaR)+dyp*sin(thetaR);
%v=0.7;
u=(1/epsilon)*(-dxp*sin(thetaR)+dyp*cos(thetaR));
public_vars.motion_vector =kinematics(v,u);
if public_vars.lost==1 
    public_vars.motion_vector =kinematics(0.5,0);
end
lidar=read_only_vars.lidar_distances;
if lidar(1)<0.2 || lidar(2)<0.2 || lidar(8)<0.2
    public_vars.motion_vector=[-public_vars.motion_vector(1),public_vars.motion_vector(2)];
elseif lidar(3)<0.2 || lidar(4)<0.2
    public_vars.motion_vector=[public_vars.motion_vector(1),public_vars.motion_vector(2)/2];
elseif lidar(6)<0.2 || lidar(7)<0.2
    public_vars.motion_vector=[public_vars.motion_vector(1)/2,public_vars.motion_vector(2)];
elseif lidar(5)<0.2 || lidar(4)<0.2 || lidar(6)<0.2
    public_vars.motion_vector=[abs(public_vars.motion_vector(1)),abs(public_vars.motion_vector(2))];
end
end