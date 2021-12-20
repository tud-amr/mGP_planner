function trajectory = solve_trajectory(trajectory)
%SOLVE_TRAJECTORY 

% Solve each dimension separately.
% Segments.coefficients [N x K] per vertex.
% Segments.time [n_vertices] array.

%for i = 1 : trajectory.num_elements
  for j = 1 : trajectory.K
    coefficients = solve_problem(trajectory.vertices_structs(j));
    % Split up the coefficients into segments.
    for i = 1 : trajectory.num_elements-1
      start_index = (i-1) * (trajectory.N + 1) + 1;
      end_index = i * (trajectory.N + 1);
      trajectory.segments(i).coefficients(:, j) = coefficients(start_index:end_index);
      trajectory.segments(i).time = trajectory.vertices_structs(j).times(i+1);
    end
  end
%end

end

