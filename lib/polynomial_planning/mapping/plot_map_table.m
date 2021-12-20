function  plot_map_table(map, map_table)
if (nargin < 2)
  map_table = get_map_table(map);
end
%PLOT_MAP_TABLE Summary of this function goes here
%   Detailed explanation goes here
imagesc(map.XWorldLimits, fliplr(map.YWorldLimits), map_table);
set(gca, 'Ydir', 'normal');
axis image;
end

