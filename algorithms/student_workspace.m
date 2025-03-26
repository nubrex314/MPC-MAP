function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
%% Week 2
%Task 2
%public_vars = senzor_measure_week2(read_only_vars, public_vars);
%%
% 8. Perform initialization procedure
if read_only_vars.counter<2
     public_vars = init_kalman_filter(read_only_vars, public_vars);
     public_vars = init_particle_filter(read_only_vars, public_vars);
     return;
end
if ~isnan(read_only_vars.gnss_position)
    if public_vars.kf_enabled==0 && public_vars.init_iterations<20
        [public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
        public_vars.init_iterations=public_vars.init_iterations+1;
        public_vars.motion_vector=[0,0];
        return;
    else
         public_vars.init_iterations=1;
         public_vars.kf_enabled=1;
         public_vars.pf_enabled=0;
    end
else
        if public_vars.pf_enabled==0 && public_vars.init_iterations<20
                public_vars.particles = update_particle_filter(read_only_vars, public_vars);
                public_vars.init_iterations=public_vars.init_iterations+1;
                %public_vars.motion_vector=[0.2,0.1];
                return;
        else
               public_vars.init_iterations=1;
               public_vars.pf_enabled=1;
               public_vars.kf_enabled=0;
        end
end  
% 9. Update particle filter
if public_vars.pf_enabled
public_vars.particles = update_particle_filter(read_only_vars, public_vars);
end
% 10. Update Kalman filter
if public_vars.kf_enabled
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);
end

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning

if isempty(public_vars.path)
    public_vars.path = plan_path(read_only_vars, public_vars);
else
   distance = norm(public_vars.estimated_pose(1:2)-public_vars.path(1,:));    
if distance>2
   public_vars.path = plan_path(read_only_vars, public_vars);
end
end

% 13. Plan next motion command
if read_only_vars.counter>20
public_vars = plan_motion(read_only_vars, public_vars);
end
end
