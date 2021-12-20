function [pos_closest, idx] = find_closest_point(pos, points)

    num_points = size(points, 1);
    min_dis = 100;
    for i = 1 : num_points
        pos_val = points(i, :);
        dis = norm(pos - pos_val);
        if dis < min_dis
            idx = i;
            pos_closest = pos_val;
            min_dis = dis;
        end
    end
end 