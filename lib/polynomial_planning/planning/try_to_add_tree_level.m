function better_nodes = try_to_add_tree_level(map, goal_point, nodes, tree_level, new_tree_level)

nodes_at_level = find(nodes(:, 3) == tree_level);

% Derp no clue what to do about the rest.
sample_radius = 2;
num_samples = 5;

% Can actually preallocate.
better_nodes = [];
% Get the collision point to the goal (we can only do this once in an
% 'efficient' implementation.
for i = 1:length(nodes_at_level)
  node = nodes(nodes_at_level(i), :);
  start_point = node(1:2);

  straight_collisions = is_line_in_collision(map, start_point, goal_point);

  if (isempty(straight_collisions))
    continue;
  end
  
  collision_point = straight_collisions(1, :);
  path_direction_vector = (collision_point - start_point)/norm(start_point - collision_point);
  
  % Hack - bisect instead.
  %collision_point = (goal_point - start_point)/2 + start_point;

  samples = get_samples(collision_point, path_direction_vector, sample_radius, num_samples);
  
  % Check samples for validity.
  % TODO: wrap this in a function.
  valid_samples = [];
  for j = 1:size(samples, 1)
    if ~is_point_in_collision(map, samples(j, :))
      valid_samples(end+1, :) = samples(j, :);
    end
  end
  
  new_nodes_temp = try_to_add_nodes(map, start_point, valid_samples, new_tree_level, nodes_at_level(i));
  better_nodes = vertcat(better_nodes, new_nodes_temp);
end
end