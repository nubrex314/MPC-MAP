function [public_vars,particle] = init_particle_filter(read_only_vars, public_vars,n)
%INIT_PARTICLE_FILTER Summary of this function goes here
if ~isempty(read_only_vars.map.gnss_denied)
    if nargin < 3
        n=1000;
        public_vars.particles = [read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                                read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                                2*pi*rand(n,1)];  
    else
        particle = [read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                     read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                     2*pi*rand(n,1)];   
        for i =1:n
            if particle(i,1)< read_only_vars.map.limits(1)|| particle(i,1)> read_only_vars.map.limits(3)
                particle(i,:)=[read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                     read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                     2*pi*rand(n,1)];
                i=i-1;
            end
            if particle(i,2)< read_only_vars.map.limits(2)|| particle(i,2)> read_only_vars.map.limits(4)
                particle(i,:)=[read_only_vars.map.gnss_denied(1) + (read_only_vars.map.gnss_denied(3) - read_only_vars.map.gnss_denied(1)) *rand(n,1), ...   
                     read_only_vars.map.gnss_denied(2) + (read_only_vars.map.gnss_denied(6) - read_only_vars.map.gnss_denied(2)) *rand(n,1), ...   
                     2*pi*rand(n,1)];
                i=i-1;
            end
        end
    end
else
    public_vars.particles = [];
end

end

