function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here
%% Week 2
%Task 2
soubor='senzor_data.mat';
if read_only_vars.counter<500
    if isfile(soubor)
         data = load(soubor);
         dataset=data.dataset;
    else
    dataset = []; % Pokud soubor neexistuje, vytvoříme nový dataset
    end
    dataset=[dataset;read_only_vars.gnss_position(1,1:end), read_only_vars.lidar_distances(1,1:end)];
    save(soubor, 'dataset');
else
    dataset = load(soubor);
    sigma=std(dataset.dataset)
    mu=mean(dataset.dataset)
        figure(2)
        subplot(2,1,1)
    histogram(dataset.dataset(1:end,1))
    title('Souradnice X');
    xlabel('Hodnota');
    ylabel('Četnost');
        subplot(2,1,2)
    histogram(dataset.dataset(1:end,2))
    title('Souradnice Y');
    xlabel('Hodnota');
    ylabel('Četnost');

        figure(3)
    for i=1:8
    subplot(2,4,i)
    histogram(dataset.dataset(1:end,i+2))
    title(['Senzor ', num2str(i)]);
    xlabel('Vzdalenost');
    ylabel('Četnost');
    end
    error("konec mereni")
end
%%
% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
public_vars.estimated_pose = estimate_pose(public_vars); % (x,y,theta)

% 12. Path planning
public_vars.path = plan_path(read_only_vars, public_vars);

% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);



end

