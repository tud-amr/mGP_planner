%% Create a random map.
map_size = 1;

if (map_size == 2)
  map = create_random_map(10, 10, 10, 50);
  start_point = [0.5 0.5];
  goal_point = [9.5 9.5];
elseif (map_size == 3)
  map = create_random_map(2, 2, 10, 10);
  start_point = [0.5 0.5];
  goal_point = [1.5 1.5];
elseif (map_size == 1)
  clf;

  map = create_random_map(4, 4, 10, 10, 0.4);  
  %map = robotics.BinaryOccupancyGrid(4, 5, 10);
  %setOccupancy(map, [2, 1.7], 1);
  %inflate(map, 0.4);

  start_point = [0.5 0.5];
  goal_point = [3.5 3.5];
end


%% Clear 0.5, 0.5 and 3.5, 3.5
setOccupancy(map, vertcat(start_point, goal_point, ...
  start_point+0.05, goal_point+0.05, start_point-0.05, goal_point-0.05), 0);


%% Get a straight line plan.
%path = [start_point; goal_point];

K = 2; % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code
v_max = 1.0;
a_max = 2.0;

trajectory = create_trajectory(K, N);
trajectory = add_vertex_to_trajectory(trajectory, start_point, 1);

trajectory = add_vertex_to_trajectory(trajectory, 1*(start_point+goal_point)/4, 0, 1);
trajectory = add_vertex_to_trajectory(trajectory, 2*(start_point+goal_point)/4, 0, 1);
trajectory = add_vertex_to_trajectory(trajectory, 3*(start_point+goal_point)/4, 0, 1);

%trajectory = add_vertex_to_trajectory(trajectory, 3*(start_point+goal_point)/4, 0, 1);

trajectory = add_vertex_to_trajectory(trajectory, goal_point, 1);

% Estimate segment times.
trajectory = estimate_trajectory_times(trajectory, v_max, a_max);
trajectory = solve_trajectory(trajectory);

%trajectory = plan_path_waypoints([start_point;goal_point]);

%% Optimize path around obstacles.
%trajectory_opt = optimize_path_collisions(trajectory);

%% Plot
[cost_map, cost_map_x, cost_map_y] = get_cost_map(map);
plot_map_table(map, cost_map);
colorbar;
hold on;
plot([start_point(1), goal_point(1)], [start_point(2), goal_point(2)], 'xw');
[t, p] = sample_trajectory(trajectory, 0.1);
plot(p(:, 1), p(:, 2), 'w');

%hold off;
