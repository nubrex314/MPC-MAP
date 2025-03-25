function [path] = astar(read_only_vars, public_vars)
    map=read_only_vars.discrete_map.map;
    [rows, cols] = size(map);

    map=Dilating_obstacles(map, 0.5,read_only_vars);

    goal_pos=[read_only_vars.discrete_map.goal(2),read_only_vars.discrete_map.goal(1)];

    %[rows, cols] = size(map);
    start_pos=[round(rows/read_only_vars.map.limits(3)*public_vars.estimated_pose(1)),round(cols/read_only_vars.map.limits(4)*public_vars.estimated_pose(2))]; 
    %start_pos=[2,2];
    moves = [0 1; 1 0; 0 -1; -1 0];
    open_list = [];
    closed_list = false(rows, cols);

    g_cost = inf(rows, cols);
    f_cost = inf(rows, cols);
    parent = zeros(rows, cols, 2);

    g_cost(start_pos(1), start_pos(2)) = 0;
    f_cost(start_pos(1), start_pos(2)) = heuristic(start_pos, goal_pos);
    open_list = [start_pos, f_cost(start_pos(1), start_pos(2))];
    
    while ~isempty(open_list)
        [~, idx] = min(open_list(:,3));
        current = open_list(idx, 1:2);
        open_list(idx, :) = [];
        closed_list(current(1), current(2)) = true;

        if isequal(current, goal_pos)
            path = reconstruct_path(parent, start_pos, goal_pos);
            return;
        end

        for i = 1:size(moves, 1)
            neighbor = current + moves(i, :);

            if neighbor(1) < 0.1 || neighbor(1) > rows || ...
               neighbor(2) < 0.1 || neighbor(2) > cols
                continue;
            end

            if map(neighbor(1), neighbor(2)) == 1 || closed_list(neighbor(1), neighbor(2))
                continue;
            end

            tentative_g = g_cost(current(1), current(2)) + 1;
            
            if tentative_g < g_cost(neighbor(1), neighbor(2))
                parent(neighbor(1), neighbor(2), :) = current;
                g_cost(neighbor(1), neighbor(2)) = tentative_g;
                f_cost(neighbor(1), neighbor(2)) = tentative_g + heuristic(neighbor, goal_pos) + 100*map(neighbor(1), neighbor(2)) ;

                if ~any(ismember(open_list(:,1:2), neighbor, 'rows'))
                    open_list = [open_list; neighbor, f_cost(neighbor(1), neighbor(2))];
                end
            end
        end
    end
    path = [];
    disp("No Path")
end

function h = heuristic(pos, goal)
    h = norm(pos-goal);
end

function path = reconstruct_path(parent, start, goal)
    path = goal;
    while ~isequal(path(1,:), start)
        prev = parent(path(1,1), path(1,2), :);
        path = [prev(:)'; path];
    end
end

function map_ex = Dilating_obstacles(map, d,read_only_vars)
d= ceil(read_only_vars.discrete_map.dims(1) / read_only_vars.map.limits(3) * d );
[nRows, nCols] = size(map);
for i = 1:nRows
    for j = 1:nCols
        if map(i, j) == 1
            map_ex(i, j) = 1;
        else
            minDist = inf; 
            for di = -d:d
                for dj = -d:d
                    if i+di > 0 && i+di <= nRows && j+dj > 0 && j+dj <= nCols
                        if map(i+di, j+dj) == 1
                            dist = abs(di) + abs(dj);
                            minDist = min(minDist, dist);
                        end
                    end
                end
            end
            
            if minDist <= d
                map_ex(i, j) = 1 - (minDist / d);
            else
                map_ex(i, j) = 0;
            end
        end
    end
end
end