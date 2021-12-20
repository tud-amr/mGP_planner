function trajectory = set_vertex_derivatives(trajectory, index, v, a)
%SET_VERTEX_DERIVATIVES 
% Requires that the vertex is already held fixed (start or end only).

if index > trajectory.num_elements
  disp 'Not enough elements!'
  return
end

for i = 1 : trajectory.K
  trajectory.vertices_structs(i).vertices(index).constraints(2, 2) = v(i);
  trajectory.vertices_structs(i).vertices(index).constraints(3, 3) = a(i);
end

end

