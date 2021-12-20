function [map_ros] = create_forest_map(type, id, map_size, tree_size)
if (nargin < 1)
  type = 0;
  id = 1;
end
if (nargin < 3)
  map_size = 5;
end
if (nargin < 4)
  tree_size = 1;
end

if (type == 0)
  map_table = getForestMap(id);
else
  map_table = getForestMapPoisson(id, map_size, tree_size);
end

res = map_size/size(map_table, 1);

map_ros = robotics.BinaryOccupancyGrid(1-map_table, 1/res);

%show(map_ros)
end
