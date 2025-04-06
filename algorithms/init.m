function [public_vars] = init(public_vars,read_only_vars)

    if read_only_vars.counter<=1
        public_vars.change=0;
        public_vars.resablicng_per=0.90;
        public_vars.lost=1;
        public_vars.path_idx=0;
        public_vars.outin_count=1;
        
         if ~isnan(read_only_vars.gnss_position)
            public_vars.kf_enabled=1;
            public_vars.estimated_pose = [mean(read_only_vars.gnss_history(:,1)) mean(read_only_vars.gnss_history(:,2)) 0];
            public_vars = init_kalman_filter(read_only_vars, public_vars);
         else
             public_vars = init_kalman_filter(read_only_vars, public_vars);
         end
        return;
    end
    if ~isnan(read_only_vars.gnss_position)
        public_vars = init_kalman_filter(read_only_vars, public_vars);
        public_vars.kf_enabled=1;
        public_vars.kidnapped=0;
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = estimate_pose(public_vars); 
        public_vars = init_particle_filter(read_only_vars, public_vars,1000);
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        public_vars.estimated_pose = [public_vars.mu(1),public_vars.mu(2),median(public_vars.particles(1:1000*public_vars.resablicng_per,3))];
        public_vars.mu=public_vars.estimated_pose;
    else
        public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)
        public_vars.pf_enabled=1;
        public_vars = init_particle_filter(read_only_vars, public_vars);
        public_vars.kidnapped=1;
        public_vars.particles = update_particle_filter(read_only_vars, public_vars);
        public_vars.outin_count=1;
        public_vars.motion_vector=[0.2,-0.2];
        return;
    end
end