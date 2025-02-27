function [path] = indor_1_path
[x_arc1,y_arc1]=generate_arc([2,8.5],[6,4.5],-1,4,15);
[x_line1, y_line1] = generate_line([6,4.5], [6,3], 3);
[x_arc2,y_arc2]=generate_arc([6,3],[9,3],1,1.5,10);
[x_line2, y_line2] = generate_line([9,3], [9,9], 15);
x_path= [
     x_arc1;
     x_line1;
     x_arc2;
     x_line2;
];
y_path= [
     y_arc1;
     y_line1;
     y_arc2;
     y_line2;
];
path=[x_path,y_path];
end

function [x_arc, y_arc] = generate_arc(start_point, end_point, direction,radius, num_points)
    mid_x = (start_point(1) + end_point(1)) / 2;
    mid_y = (start_point(2) + end_point(2)) / 2;

    dx = end_point(1) - start_point(1);
    dy = end_point(2) - start_point(2);
    length_half = sqrt(dx^2 + dy^2)/2;

    height = sqrt(radius^2 - length_half^2);
    perpendicular = [-dy, dx] / norm([dx, dy]);

    % Determine the center of the arc
    arc_center = [mid_x, mid_y] + direction * height * perpendicular;

    % Compute angles for arc generation
    start_angle = atan2(start_point(2) - arc_center(2), start_point(1) - arc_center(1));
    end_angle = atan2(end_point(2) - arc_center(2), end_point(1) - arc_center(1));

    % Ensure proper angle sweep direction
    if direction == 1 && end_angle < start_angle
        end_angle = end_angle + 2 * pi;
    elseif direction == -1 && end_angle > start_angle
        end_angle = end_angle - 2 * pi;
    end

    theta = linspace(start_angle, end_angle, num_points);
    x_arc = arc_center(1) + radius * cos(theta);
    x_arc=x_arc';
    y_arc = arc_center(2) + radius * sin(theta);
    y_arc =y_arc';
end

function [x_path, y_path] = generate_line(start_point, end_point, num_points)
        x_path = linspace(start_point(1), end_point(1), num_points);
        x_path=x_path';
        y_path = linspace(start_point(2), end_point(2), num_points);
        y_path=y_path';
end


