function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Summary of this function goes here
n=length(weights);
index=randi([1,(n-1)]);
wmax=max(weights);
for i=1:n
    beta=rand(1)*2*wmax;
    while weights(index)<beta
        beta=beta-weights(index);
        index=index+1;
        if index>n
            index=1;
        end
    end
    new_particles(i,:) = particles(index,:);
end
end

