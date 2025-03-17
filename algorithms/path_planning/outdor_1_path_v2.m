function [path] = outdor_1_path_v2
[x_arc1,y_arc1]= generate_arc([2,2],[4,4],1,2,10);
[x_arc21, y_arc21] = generate_arc([4,4],[6,6],-1,2,10);
[x_arc2, y_arc2] = generate_arc([6,6],[8,8],1,2,10);
[x_line1, y_line1] = generate_line([8,8], [8,14.5], 25);
[x_line12, y_line12] = generate_line([8,14.5], [19,14.5], 25);
[x_line13, y_line13] = generate_line([19,14.5], [19,8], 25);
[x_line14, y_line14] = generate_line([19,8], [13,8], 25);
[x_line2, y_line2] = generate_line([13,8], [13,4], 15);
[x_line3, y_line3] = generate_line([13,4], [16,4], 15);
[x_line4, y_line4] = generate_line([16,4], [16,1], 15);
x_path= [
     x_arc1;
     x_arc21
     x_arc2
     x_line1
     x_line12
     x_line13
     x_line14
     x_line2
     x_line3
     x_line4
];
y_path= [
     y_arc1;
     y_arc21
     y_arc2
     y_line1
     y_line12
     y_line13
     y_line14
     y_line2
     y_line3
     y_line4
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


