function [trajectory] = plan_yaw_waypoints(waypoints, segment_time)
%PLAN_PATH_WAYPOINTS

% Waypoints is W x K (K = 1 only for yaw).
K = size(waypoints, 2); % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code

num_waypoints = size(waypoints, 1);
trajectory = create_trajectory(K, N);

for i = 1:num_waypoints
  start_or_stop = 0;
  if (i == 1 || i == num_waypoints)
    start_or_stop = 1;
  end
  trajectory = add_vertex_to_trajectory(trajectory, waypoints(i, :)', start_or_stop);
end

% Set trajectory segment time
for i = 2 : trajectory.num_elements
  time = segment_time(i);
  trajectory = set_trajectory_segment_time(trajectory, i, time);
end

% Now solve the problem...
trajectory = solve_trajectory(trajectory);

end
