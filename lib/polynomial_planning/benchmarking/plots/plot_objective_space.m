%show(map_ros);
%%
num_segments = 2;

K = 2; % Number of dimensions.
N = 11; % Is equivalent to N = 12 in Markus code
v_max = 1.0;
a_max = 2.0;

start_point = [0.5 0.5];
goal_point = [4.5, 4.5];

trajectory = create_trajectory(K, N);
trajectory = add_vertex_to_trajectory(trajectory, start_point, 1);

max_i = num_segments - 1;

for i = 1:max_i
  trajectory = add_vertex_to_trajectory(trajectory, start_point + i*(-start_point+goal_point)/num_segments, 0, 1);
  trajectory = set_trajectory_segment_time(trajectory, i+1, 10/num_segments);
end
trajectory = add_vertex_to_trajectory(trajectory, goal_point, 1);
trajectory = set_trajectory_segment_time(trajectory, i+2, 10/num_segments);

trajectory = solve_trajectory(trajectory);

%%

Ms = {};
D_Fs = {};
A_invs = {};
R_unordereds = {};
x0 = [];
for k = 1:trajectory.K
  [Ms{k}, D_Fs{k}] = assemble_M_and_D_F(trajectory.vertices_structs(k));
  [A_invs{k}, R_unordereds{k}] = assemble_A_and_R(trajectory.vertices_structs(k).times, trajectory.N);

  total_ds = size(Ms{k}, 2);

  n = trajectory.N + 1;

  p = [];
  for i = 1:length(trajectory.segments)
    p((i-1)*n+1:i*n, 1) = trajectory.segments(i).coefficients(:, k);
  end
  total_num_fixed = trajectory.vertices_structs(k).num_fixed;
  M = Ms{k};
  A_inv = A_invs{k};
  D_P = pinv(M)*inv(A_inv)*p;
  D_P = D_P(total_num_fixed+1:total_ds);
  x0(end+1:end+length(D_P)) = D_P;
end

x0

[cost_map, cost_map_x, cost_map_y] = get_cost_map(map);

cost_func = @(x0)get_trajectory_cost(x0, trajectory, map,  cost_map, cost_map_x, cost_map_y, Ms, D_Fs, A_invs, R_unordereds);

[cost, grad] = cost_func(x0);

%%
%vals = 1:0.1:4;

vals = -2:0.1:2;


[x02, x010] = meshgrid(vals, vals);

cost_map_vel = zeros(size(x02));

for i = 1:size(x02, 1)
  for j = 1:size(x02, 2)
    x = x0;
    x(4) = x02(i, j);
    x(12) = x010(i, j);
    cost_map_vel(i, j) = cost_func(x);
  end
end

%[trajectory_out, iterations, final_cost, initial_cost] = optimize_trajectory_collisions(map, trajectory, 1, 0, 1);

%%
%plot_map_table(map, map_table);
%hold on;
%hold off;

