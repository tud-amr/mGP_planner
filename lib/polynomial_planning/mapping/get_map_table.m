function obstacle_map = get_map_table(map)
%GET_SUBMAP Summary of this function goes here
%   Detailed explanation goes here

% Create a map from the binary occupancy grid.
msg = rosmessage('nav_msgs/OccupancyGrid');
writeBinaryOccupancyGrid(msg, map);
obstacle_map = flipud(reshape(msg.Data, [map.GridSize(2), map.GridSize(1)])');
obstacle_map = obstacle_map/max(max(obstacle_map));
end