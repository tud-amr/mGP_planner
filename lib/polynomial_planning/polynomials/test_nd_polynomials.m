% Sample scenario - go from (0, 0) to (1, 1) in a straight line.
% Here, N is the degree, in Markus's C implementation N is degree + 1 (so
% the number of coefficients).

K = 2; % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code
v_max = 1.0;
a_max = 2.0;

% Trajectory is a struct containing K vertex structs containing the
% vertices.
clear trajectory;
trajectory = create_trajectory(K, N);

% Add all the points.
trajectory = add_vertex_to_trajectory(trajectory, [0, 0, 0], 1);
trajectory = add_vertex_to_trajectory(trajectory, [0.7, -0.5, 0], 0);
trajectory = add_vertex_to_trajectory(trajectory, [0.5, -0.3, 0], 0);
trajectory = add_vertex_to_trajectory(trajectory, [0.7, 1, 0], 0);
trajectory = add_vertex_to_trajectory(trajectory, [1, 2, 0], 1);

% Estimate segment times.
trajectory = estimate_trajectory_times(trajectory, v_max, a_max);

% Now solve the problem...
trajectory = solve_trajectory(trajectory);

% Now sample it.
[t, p, v, a] = sample_trajectory(trajectory, 0.1);

% And finally plot it!
plot(t, p);
plot(p(:,1), p(:,2))
