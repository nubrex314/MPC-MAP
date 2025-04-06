function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
% Init
if read_only_vars.counter<20
public_vars=init(public_vars,read_only_vars);
return;
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

%find after change
if public_vars.change>=1

    if public_vars.change>=10
        public_vars.change=0;
        return;
    end
    public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
    public_vars = plan_motion(read_only_vars, public_vars);
    public_vars.change=public_vars.change+1;

    if public_vars.pf_enabled
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        return;
    elseif public_vars.kf_enabled
        public_vars.mu=public_vars.estimated_pose';
        init_kalman_filter(read_only_vars,public_vars);
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.motion_vector=[0,0];
        return;
    end
end

% Particle filter
if public_vars.pf_enabled
    if norm(public_vars.estimated_pose(1:2)-read_only_vars.map.goal)<0.2
        public_vars = init_particle_filter(read_only_vars, public_vars);
    end

    public_vars.particles = update_particle_filter(read_only_vars, public_vars);

    if public_vars.outin_count<1500
        public_vars.outin_count=public_vars.outin_count+1;
    end

    if std(public_vars.particles(1:1000*public_vars.resablicng_per,1))<1.5
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
    public_vars.path_idx=1;
else
   distance = norm(public_vars.estimated_pose(1:2)-public_vars.path(public_vars.path_idx,:));    
    if distance>1
       public_vars.path = plan_path(read_only_vars, public_vars);
       public_vars.path_idx=1;
    end
end

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);

end
