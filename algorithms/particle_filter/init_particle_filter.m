function [public_vars] = init_particle_filter(read_only_vars, public_vars)
%INIT_PARTICLE_FILTER Summary of this function goes here
    max_value = max(read_only_vars.map.limits); 
n=800;
public_vars.particles = [max_value*rand(n,1), ...   
        max_value*rand(n,1), ...   
        6.28*rand(n,1)];  
end

