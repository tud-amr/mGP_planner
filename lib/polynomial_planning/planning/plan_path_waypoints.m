function [trajectory] = plan_path_waypoints(waypoints, max_vel, max_acc, ...
    start_derivatives, end_derivatives)
%PLAN_PATH_WAYPOINTS

% Waypoints is W x K (K = 2 for 2D).
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

% Fill in start derivatives and end derivatives if provided.
% Start derivatives: 3 x K (ignore past acceleration for now?)
if nargin > 3
  trajectory = set_vertex_derivatives(trajectory, 1, start_derivatives(1, :), start_derivatives(2, :));
end
if nargin > 4
  trajectory = set_vertex_derivatives(trajectory, trajectory.num_elements, end_derivatives(1, :), end_derivatives(2, :));  
end
  
% Estimate segment times.
trajectory = estimate_trajectory_times(trajectory, max_vel, max_acc);

% Now solve the problem...
trajectory = solve_trajectory(trajectory);

end

