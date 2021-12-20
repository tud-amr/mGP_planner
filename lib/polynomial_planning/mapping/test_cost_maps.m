
map = robotics.BinaryOccupancyGrid(4, 5, 10);
setOccupancy(map, [2, 1.7], 1);
setOccupancy(map, [3, 2.5], 1);

inflate(map, 0.4);

start_point = [0.5 0.5];
goal_point = [3.5 3.5];

[cost_map, cost_map_x, cost_map_y] = get_cost_map(map);
%cost_map_y = cost_map_y;
plot_map_table(map, cost_map);
colorbar;

% Verify that the cost maps match the derivatives.
test_point = [1.75, 1.25];
h = 0.001;

value_at_point = get_table_value_at_pos(test_point, map, cost_map)
grad_x = (get_table_value_at_pos(test_point + [h, 0], map, cost_map) ...
  - get_table_value_at_pos(test_point - [h, 0], map, cost_map))/(2*h);
grad_y = (get_table_value_at_pos(test_point + [0, h], map, cost_map) ...
  - get_table_value_at_pos(test_point - [0, h], map, cost_map))/(2*h);

grad_map_x = get_table_value_at_pos(test_point, map, cost_map_x);
grad_map_y = get_table_value_at_pos(test_point, map, cost_map_y);

%% Let's generate the actual costmap
[X, Y] = meshgrid(0:0.01:4, 0:0.01:5);

val_grad_x = zeros(size(X));
val_grad_y = zeros(size(X));
val_val = zeros(size(X));

for i = 1:size(X, 1)
  for j = 1:size(X, 2)
    test_point = [X(i, j), Y(i, j)];
    val_val(i, j) = get_table_value_at_pos(test_point, map, cost_map);
    val_grad_x(i, j) = get_table_value_at_pos(test_point, map, cost_map_x);
    %val_grad_y(i, j) = get_table_value_at_pos(test_point, map, cost_map_y);
  end
end

%%
surf(val_val, 'EdgeColor','none')
