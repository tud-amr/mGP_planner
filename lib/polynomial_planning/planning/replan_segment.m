function [vertices, trajectory] = replan_segment(map, start_point, goal_point)
%REPLAN_SEGMENT

% First, check the straight-line trajectory between the points.
collisions = is_line_in_collision(map, start_point, goal_point);

if isempty(collisions)
  return;
end

% Start building the node tree, with the start point. :)
% Also keep track of tree level. Parent is also tree level 0.
% Start node has no parent! It is the ultimate parent.
% [x y tree_level parent]
nodes = [start_point(1), start_point(2), 0, 0];

tree_level = 0;
max_tree_level = 3;
parent = 1;

valid_indices = [];
while (isempty(valid_indices) && tree_level <= max_tree_level)
  % Split!
  % Pre-split may require that tree levels aren't monotonically increasing
  % toward the goal, so specify both current and next for now.
  new_nodes = try_to_add_tree_level(map, goal_point, nodes, tree_level, tree_level + 1);
  if (isempty(new_nodes))
    % Pre-split happens here, shitty shit.
    disp 'Shitty shit'
  end
  tree_level = tree_level + 1;
  nodes = vertcat(nodes, new_nodes);
  valid_indices = connect_to_goal(map, goal_point, nodes, tree_level);
end

hold on;
% Generate some sample trajectories.
for i = 1:length(valid_indices)
  % Recurse back through parents.
  vertices = [goal_point];
  parent_index = valid_indices(i);
  while parent_index ~= 0
    vertices(end+1, :) = nodes(parent_index, 1:2);
    parent_index = nodes(parent_index, 4);
  end
  trajectory = plan_path_waypoints(vertices);
  [t, p] = sample_trajectory(trajectory, 0.2);
  plot(p(:, 1), p(:, 2), 'r');
  plot(vertices(:, 1), vertices(:, 2), 'g');
end
hold off;

end
