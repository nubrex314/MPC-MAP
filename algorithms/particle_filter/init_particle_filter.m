function [public_vars,particle] = init_particle_filter(read_only_vars, public_vars,n)
%INIT_PARTICLE_FILTER Summary of this function goes here
if ~isempty(read_only_vars.map.gnss_denied)
    if nargin < 3
        n=1000;
        public_vars.particles = [read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                                read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                                2*pi*rand(n,1)];  
    else
        if public_vars.kidnapped==0
            for i =1:n
                particle(i,:)=randomPose(public_vars.estimated_pose(1),public_vars.estimated_pose(2),public_vars.outin_count/20+0.5);
                %particle(i,:)=randomPose(public_vars.estimated_pose(1),public_vars.estimated_pose(2),0.5);
            end
            public_vars.particles =particle;
        else
        particle = [read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                     read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                     2*pi*rand(n,1)];  
         public_vars.particles =particle;
        end
    end
else
    public_vars.particles = [];
end

end

function [particle] = randomPose(X0, Y0, d_max)
    X_new = X0 + (rand() * 2 - 1) * d_max;
    Y_new = Y0 + (rand() * 2 - 1) * d_max;
    theta_new = rand() * 2 * pi;
    particle=[X_new,Y_new,theta_new];
end
