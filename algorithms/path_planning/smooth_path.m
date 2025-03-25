function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here
alfa=0.5;
beta=0.5;
iterations= 2;
new_path = old_path;  
    for iter = 1:iterations
        for i = 2:size(new_path,1)-1
            for j = 1:2 
                new_path(i,j) = new_path(i,j) + alfa * (new_path(i,j) - new_path(i,j)) ...
                                    + beta * (new_path(i-1,j) + new_path(i+1,j) - 2 * new_path(i,j));
            end
        end
    end
end
