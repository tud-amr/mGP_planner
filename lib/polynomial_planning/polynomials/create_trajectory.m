function trajectory = create_trajectory(K, N)
trajectory.N = N;
trajectory.K = K;
trajectory.num_elements = 0;

for i = 1:trajectory.K
  %trajectory.vertices_structs(i).num_fixed = 0;
  %trajectory.vertices_structs(i).num_elements = 0;
  %trajectory.vertices_structs(i).N = N;
  %trajectory.vertices_structs(i).times = [];
  %trajectory.vertices_structs(i).vertices = struct();
end
end