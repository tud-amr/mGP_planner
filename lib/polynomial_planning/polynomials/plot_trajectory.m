function plot_trajectory(trajectory)
  % get the color of the next thing.
  color_order = get(gca, 'ColorOrder');
  next_color =  color_order(mod(length(get(gca, 'Children')), size(color_order, 1)), :);
  
  % Sample the trajectory.
  [t, p] = sample_trajectory(trajectory, 0.1);
  
  % Get the vertices.
  verts = zeros(trajectory.num_elements, trajectory.K);
  for i = 1:trajectory.num_elements
    verts(i, :) = get_vertex_trajectory(trajectory, i);
  end
  
  plot(p(:, 1), p(:, 2), 'Color', next_color, 'LineWidth', 2);
  plot(verts(:, 1), verts(:, 2), 'o', 'Color', next_color, 'LineWidth', 2);
end