function trajectory = set_trajectory_segment_time(trajectory, index, time)
for i = 1 : trajectory.K
  trajectory.vertices_structs(i).times(index) = time;
end

end