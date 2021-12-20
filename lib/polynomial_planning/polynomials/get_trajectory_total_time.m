function total_time = get_trajectory_total_time(trajectory)
total_time = 0;
for i = 1 : length(trajectory.segments)
  total_time = total_time + trajectory.segments(i).time;
end
end