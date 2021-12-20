function valid_indices = connect_to_goal(map, goal_point, nodes, tree_level)
%CONNECT_TO_GOAL
valid_indices = [];

% Select nodes on the correct tree level.
nodes_at_level = find(nodes(:, 3) == tree_level);
for i = 1 : length(nodes_at_level)
  node = nodes(nodes_at_level(i), :);
  
  straight_collisions = is_line_in_collision(map, node(1:2), goal_point);
  if isempty(straight_collisions)
    valid_indices(end+1) = nodes_at_level(i);
  end
end
end

