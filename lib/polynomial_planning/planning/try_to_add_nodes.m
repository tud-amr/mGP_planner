function new_nodes = try_to_add_nodes(map, start_point, samples, tree_level, parent)
%TRY_TO_ADD_NODES 

new_nodes = [];
for i = 1:size(samples, 1)
  straight_collisions = is_line_in_collision(map, start_point, samples(i, :));
  if isempty(straight_collisions)
    new_nodes(end+1, :) = [samples(i, :), tree_level, parent];
  end
end

end
