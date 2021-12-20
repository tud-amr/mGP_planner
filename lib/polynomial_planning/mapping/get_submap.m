function submap = get_submap(map, center, submap_size)
%GET_SUBMAP Summary of this function goes here
%   Detailed explanation goes here

% Create a map from the binary occupancy grid.
msg = rosmessage('nav_msgs/OccupancyGrid');
writeBinaryOccupancyGrid(msg, map);
obstacle_map = flipud(reshape(msg.Data, map.GridSize)');
%map_size = size(obstacle_map);

% Convert start and goal into grid points.
res = map.Resolution;

center_shifted(2) = center(1);
center_shifted(1) = center(2);
temp = submap_size(1);
submap_size(1) = submap_size(2);
submap_size(2) = temp;
center_shifted(1) = map.YWorldLimits(2) - center(2);

% Get the submap from the obstacle map.
top_left = (center_shifted - submap_size/2)*res;
bottom_right = (center_shifted + submap_size/2)*res;

% Make sure we don't exceed the limits.
top_left(1) = max(1, round(top_left(1)));
top_left(2) = max(1, round(top_left(2)));
bottom_right(1) = max(1, round(bottom_right(1)));
bottom_right(2) = max(1, round(bottom_right(2)));
top_left(1) = min(size(obstacle_map, 1), top_left(1));
top_left(2) = min(size(obstacle_map, 2), top_left(2));
bottom_right(1) = min(size(obstacle_map, 1), bottom_right(1));
bottom_right(2) = min(size(obstacle_map, 2), bottom_right(2));

submap_data = obstacle_map(top_left(1):bottom_right(1), top_left(2):bottom_right(2))/100;

% Create a new map from the submap.
submap = robotics.BinaryOccupancyGrid(submap_data, res);
% Location of bottom left corner...
submap.GridLocationInWorld = [center(1) - submap_size(2)/2, center(2) - submap_size(1)/2];
end

