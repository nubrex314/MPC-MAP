function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
XR=public_vars.estimated_pose(1);
YR=public_vars.estimated_pose(2);
thetaR=public_vars.estimated_pose(3);
epsilon=0.3;
% if read_only_vars.counter>100
k=1.8; 
% else
%   k=1; 
% end
%% I. Pick navigation target

%target = get_target(public_vars.estimated_pose, public_vars.path);
    if isempty(public_vars.path)
    disp('NO TRACK.');
    target=[XR,YR];
    else
    distance = norm([XR,YR]-public_vars.path(public_vars.path_idx,:));
        if distance < 0.5
            if public_vars.path_idx~=size(public_vars.path,1)
                public_vars.path_idx = public_vars.path_idx+1;
            else
                public_vars.lost=1;
            end
        end
    target = public_vars.path(public_vars.path_idx,:);
    end

%% II. Compute motion vector
xp=XR+epsilon*cos(thetaR);
yp=YR+epsilon*sin(thetaR);
dxp=k*(target(1)-xp);
dyp=k*(target(2)-yp);
v=(dxp*cos(thetaR)+dyp*sin(thetaR));%*1.8;
%v=0.7;
u=(1/epsilon)*(-dxp*sin(thetaR)+dyp*cos(thetaR));

%zpomalovani v zatackach
% if abs(u)>0.5
%     v=v/(k*2);
%     %u=u/(k*2);
% end
public_vars.motion_vector =kinematics(v,u);
if public_vars.lost==1 
    public_vars.motion_vector =[0.5,0.5];
end

lidar=read_only_vars.lidar_distances;
wall_distance=0.2;
if lidar(1)<wall_distance || lidar(2)<wall_distance || lidar(8)<wall_distance
    public_vars.motion_vector=[-public_vars.motion_vector(1),public_vars.motion_vector(2)];
elseif lidar(3)<wall_distance || lidar(4)<wall_distance
    public_vars.motion_vector=[public_vars.motion_vector(1),public_vars.motion_vector(2)/2];
elseif lidar(6)<wall_distance || lidar(7)<wall_distance
    public_vars.motion_vector=[public_vars.motion_vector(1)/2,public_vars.motion_vector(2)];
elseif lidar(5)<wall_distance %|| lidar(4)<wall_distance|| lidar(6)<wall_distance
    public_vars.motion_vector=[abs(public_vars.motion_vector(1)),abs(public_vars.motion_vector(2))];
end
end