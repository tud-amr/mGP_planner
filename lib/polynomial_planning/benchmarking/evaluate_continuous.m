function [trajectory_out, metrics] =  evaluate_continuous(map, start_point, goal_point, visualize, num_segments, random_perturb)
if (nargin < 5)
  num_segments = 1;
end
if (nargin < 6)
  random_perturb = 0;
end
if (nargin < 4)
  visualize = 1;
end


K = 2; % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code
v_max = 1.0;
a_max = 2.0;

trajectory = create_trajectory(K, N);
trajectory = add_vertex_to_trajectory(trajectory, start_point, 1);

max_i = num_segments - 1;

for i = 1:max_i
  trajectory = add_vertex_to_trajectory(trajectory, start_point + i*(-start_point+goal_point)/num_segments, 0, 1);
end
trajectory = add_vertex_to_trajectory(trajectory, goal_point, 1);

% Estimate segment times.
trajectory = estimate_trajectory_times(trajectory, v_max, a_max);
trajectory = solve_trajectory(trajectory);

%trajectory = plan_path_waypoints([start_point;goal_point]);

[trajectory_out, iterations, final_cost, initial_cost] = optimize_trajectory_collisions(map, trajectory, visualize, random_perturb, 1);
%optimize_trajectory_collisions(map, trajectory);

% Fill in the metrics directly from optimization.
metrics = create_metrics_struct;
metrics.num_iter = iterations;
metrics.final_cost = final_cost;
metrics.initial_cost = initial_cost;

% Estimate the rest.
dt = 0.01;
[t, p, v, a] = sample_trajectory(trajectory_out, dt);
v_norm = sqrt(v(:, 1).^2 + v(:, 2).^2);
metrics.length = sum(dt*v_norm);
colliding_times = is_trajectory_in_collision(map, trajectory_out);
metrics.success = isempty(colliding_times);
end

