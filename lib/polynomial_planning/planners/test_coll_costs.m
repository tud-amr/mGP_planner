do_prep = 1;
if (do_prep == 1)
  % Ugh this is useless.
  K = 2; % Number of dimensions.
  N = 11; % Is equivalent to N = 12 in Markus code
  v_max = 1.0;
  a_max = 2.0;
  num_segments = 4;

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

  [cost_map, cost_map_x, cost_map_y] = get_cost_map(map);

  cost_func = @(x0)get_trajectory_cost(x0, trajectory, map,  cost_map, cost_map_x, cost_map_y, Ms, D_Fs, A_invs, R_unordereds);
end
    
if (0)
  x0 = x0 + 0.1*(rand(size(x0))-0.5);
end

x_nom = x0;

[cost_nom, grad_nom] = cost_func(x_nom);

grad_num = zeros(size(grad_nom));
h = 0.001;

for i = 1:length(x0)
  x_per = zeros(size(x0));
  x_per(i) = h;
  cost_p = cost_func(x_nom + x_per);
  cost_n = cost_func(x_nom - x_per);
  %disp(sprintf('%f %f', cost_p, cost_n))
  grad_num(i) = (cost_p - cost_n)/(2*h);
end

grad_nom
grad_num

norm(grad_nom - grad_num)
%cost_nom