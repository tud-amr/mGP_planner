map = create_map;



start_point = [1 1];
goal_point = [8 5];

%start_point = [9*rand() 9*rand()];
%goal_point = [9*rand() 9*rand()];

visualize = 1;

figure(1)
clf;
evaluate_toolbox_chomp(map, start_point, goal_point, visualize)


figure(2)
plot_map_table(map, 1-get_map_table(map));
hold on;

K = 2; % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code
v_max = 1.0;
a_max = 2.0;

trajectory = create_trajectory(K, N);
trajectory = add_vertex_to_trajectory(trajectory, start_point, 1);

trajectory = add_vertex_to_trajectory(trajectory, start_point + 1*(-start_point+goal_point)/4, 0, 1);
trajectory = add_vertex_to_trajectory(trajectory, start_point + 2*(-start_point+goal_point)/4, 0, 1);
trajectory = add_vertex_to_trajectory(trajectory, start_point + 3*(-start_point+goal_point)/4, 0, 1);

%trajectory = add_vertex_to_trajectory(trajectory, 3*(start_point+goal_point)/4, 0, 1);

trajectory = add_vertex_to_trajectory(trajectory, goal_point, 1);

% Estimate segment times.
trajectory = estimate_trajectory_times(trajectory, v_max, a_max);
trajectory = solve_trajectory(trajectory);

%trajectory = plan_path_waypoints([start_point;goal_point]);

%optimize_trajectory_collisions
optimize_trajectory_collisions(map, trajectory);