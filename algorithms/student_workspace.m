function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
% Init
if read_only_vars.counter<5
    if read_only_vars.counter<=1
        return;
    end
    public_vars.change=0;
    public_vars.motion_vector=[0.2,-0.2];
    public_vars.resablicng_per=0.90;
    public_vars.lost=1;
    if ~isnan(read_only_vars.gnss_position)
        public_vars.kf_enabled=1;
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.kidnapped=0;
    else
        public_vars.pf_enabled=1;
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.kidnapped=1;
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
        public_vars.outin_count=1;
        return;
    end
end

% change indoor or outdoor
if ~isnan(read_only_vars.gnss_position)
    if public_vars.pf_enabled==1 && public_vars.change==0
        public_vars.change=1;
        public_vars.pf_enabled=0;
        public_vars.kf_enabled=1;
    end
else
    if public_vars.kf_enabled==1 && public_vars.change==0
        public_vars.change=1;
        public_vars.kf_enabled=0;
        public_vars.pf_enabled=1;
        public_vars.outin_count=1;
        public_vars =init_particle_filter(read_only_vars, public_vars, 1000);
    end
end
if public_vars.change>=1
    if public_vars.change>=20
        public_vars.change=0;
        return;
    end
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
    public_vars = plan_motion(read_only_vars, public_vars);
    public_vars.change=public_vars.change+1;
    if public_vars.pf_enabled
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        return;
    end
    if public_vars.kf_enabled
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        return;
    end
end
% Particle filter
if public_vars.pf_enabled
    if norm(public_vars.estimated_pose(1:2)-read_only_vars.map.goal)<0.2
        public_vars = init_particle_filter(read_only_vars, public_vars);
    end
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
    public_vars.outin_count=public_vars.outin_count+1;
    if std(public_vars.particles(1:1000*public_vars.resablicng_per,1))<2
        public_vars.lost=0;
    else
        public_vars.lost=1;
    end
end

% Update Kalman filter
if public_vars.kf_enabled
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.lost=0;
        public_vars.kidnapped=0;
end

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
% 12. Path planning

if isempty(public_vars.path)
    public_vars.path = plan_path(read_only_vars, public_vars);
else
   distance = norm(public_vars.estimated_pose(1:2)-public_vars.path(1,:));    
if distance>1
   public_vars.path = plan_path(read_only_vars, public_vars);
end
end

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);


end

% 8. Perform initialization procedure
% if read_only_vars.counter<2
%     public_vars.resablicng_per=0.90;
%     public_vars.kidnapped=1;
%     public_vars.lost=1;
%      public_vars = init_kalman_filter(read_only_vars, public_vars);
%      public_vars = init_particle_filter(read_only_vars, public_vars);
%      public_vars.estimated_pose = [1,1,0];
%      if ~isnan(read_only_vars.gnss_position)
%         public_vars.kf_enabled=1;
%         public_vars.pf_enabled=0;
%      else
%         public_vars.kf_enabled=0;
%         public_vars.pf_enabled=1;
%      end
%      return;
% end
% 
% if ~isnan(read_only_vars.gnss_position)
%         public_vars.kf_enabled=1;
%         public_vars.pf_enabled=0;
% else
%         public_vars.pf_enabled=1;
%         public_vars.kf_enabled=0;
% end
% % 9. Update particle filter
% if public_vars.pf_enabled
%     if norm(public_vars.estimated_pose(1:2)-read_only_vars.map.goal)<0.2
%         public_vars = init_particle_filter(read_only_vars, public_vars);
%     end
%         public_vars.particles = update_particle_filter(read_only_vars, public_vars);
%     if std(public_vars.particles(1:length(public_vars.particles)*public_vars.resablicng_per,1))<0.5
%         public_vars.lost=0;
%     else
%         public_vars.lost=1;
%     end
% end
% % 10. Update Kalman filter
% if ~public_vars.pf_enabled
% [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
%         public_vars.lost=0;
%         public_vars.kidnapped=0;
% end
% 
% % 11. Estimate current robot position
% public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
% % 12. Path planning
% 
% if isempty(public_vars.path)
%     public_vars.path = plan_path(read_only_vars, public_vars);
% else
%    distance = norm(public_vars.estimated_pose(1:2)-public_vars.path(1,:));    
% if distance>1
%    public_vars.path = plan_path(read_only_vars, public_vars);
% end
% end
% 
% % 13. Plan next motion command
% public_vars = plan_motion(read_only_vars, public_vars);
% 
% end
