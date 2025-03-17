%% Week 2
%Task 2
function [public_vars] = senzor_measure_week2(read_only_vars, public_vars)
soubor='senzor_data.mat';
if read_only_vars.counter<500
    if isfile(soubor)
        if read_only_vars.counter==1
            dataset = [];
            save(soubor, "dataset");
        end
         data = load(soubor);
         dataset=data.dataset;
    else
    dataset = []; % Pokud soubor neexistuje, vytvoříme nový dataset
    end
    dataset=[dataset;read_only_vars.gnss_position(1,1:end), read_only_vars.lidar_distances(1,1:end)];
    save(soubor, 'dataset');
else
    dataset = load(soubor);
    %sigma=std(dataset.dataset);
    public_vars.mu=[mean(dataset.dataset(:,1:2)),0];
    %     figure(2)
    %     subplot(2,1,1)
    % histogram(dataset.dataset(1:end,1))
    % title('Souradnice X');
    % xlabel('Hodnota');
    % ylabel('Četnost');
    %     subplot(2,1,2)
    % histogram(dataset.dataset(1:end,2))
    % title('Souradnice Y');
    % xlabel('Hodnota');
    % ylabel('Četnost');
    % 
    %     figure(3)
    % for i=1:8
    % subplot(2,4,i)
    % histogram(dataset.dataset(1:end,i+2))
    % title(['Senzor ', num2str(i)]);
    % xlabel('Vzdalenost');
    % ylabel('Četnost');
    % end
    %task 3
    public_vars.kf.Q=cov(dataset.dataset(1:end,1:2));
    %Lidar_cov=cov(dataset.dataset(1:end,3:10))
    %sqrt_std=sigma.^2
    %task 4
    % x = -1.5:0.01:1.5;
    % gnss_pdf=norm_pdf(x,0,sigma(1));
    % Lidar_pdf=norm_pdf(x,0,sigma(3));
    % figure(4)
    % plot(x,gnss_pdf)
    % hold on
    % plot(x,Lidar_pdf)
    % hold off
    % title('Noise Characteristics of Robot Sensors');
    % xlabel('x');
    % ylabel('Probability Density');
    % legend('GNSS','Lidar');
    % 
    % error("konec mereni")
end
end

function [pdf] = norm_pdf(x, mu,sigma)
    pdf=(1/(sigma*sqrt(2*pi))) * exp(-0.5*((x-mu)/sigma).^2);
end