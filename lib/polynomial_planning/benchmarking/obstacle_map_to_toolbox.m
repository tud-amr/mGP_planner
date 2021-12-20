function [map, bbox, cost_map] = obstacle_map_to_toolbox(map_ros)
map = get_blank_map;
map.resolution = 1/map_ros.Resolution;
map.origin = map_ros.GridLocationInWorld;
map.table = get_map_table(map_ros);
map.table = 1-map.table/(max(max(map.table)));
map.table = rot90(map.table, -1);

bbox = [map_ros.XWorldLimits(1) map_ros.YWorldLimits(1); map_ros.XWorldLimits(2) map_ros.YWorldLimits(2)];
% TODO: rename to cost_map_toolbox.
%cost_map = 
epsilon = 0.5;

cost_map = create_obstacle_cost_map(map, epsilon);
end

