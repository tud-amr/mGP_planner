function [p, v, a] = get_vertex_trajectory(trajectory, index)
%GET_VERTEX_TRAJECTORY
if index > trajectory.num_elements
  disp 'Not enough elements!'
  return
end

p = [];
v = [];
a = [];
for i = 1 : trajectory.K
  vertex = trajectory.vertices_structs(i).vertices(index);
  p(i) = vertex.constraints(1, 1);
  v(i) = vertex.constraints(2, 2);
  a(i) = vertex.constraints(3, 3);
end

end

