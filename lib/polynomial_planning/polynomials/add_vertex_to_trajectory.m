function trajectory = add_vertex_to_trajectory(trajectory, position, start_or_end, unconstrained)
% Check that position is K-D.
if (nargin < 3)
  start_or_end = 0;
end
if (nargin < 4)
  unconstrained = 0;
end

for i = 1 : trajectory.K
   if (start_or_end)
     vert = create_vertex_start_end(position(i), trajectory.N);
   else
     vert = create_vertex_position(position(i), trajectory.N, unconstrained);
   end
   % Fill in all times as 0 for now, these need to get estimated later.
   if trajectory.num_elements == 0
      vertex_struct.num_fixed = 0;
      vertex_struct.num_elements = 0;
      vertex_struct.N = trajectory.N;
      vertex_struct.times = [];
      
      vertex_struct = add_vertex(vertex_struct, vert, 0.0);
      trajectory.vertices_structs(i) = vertex_struct;
   else
    trajectory.vertices_structs(i) = add_vertex(trajectory.vertices_structs(i), vert, 0.0);
   end
end
trajectory.num_elements = trajectory.num_elements + 1;
end
