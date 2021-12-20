function [final_path, metrics] = evaluate_toolbox_chomp(map_ros, start, goal, visualize)
% CHOMP CHOMP CHOMP

% Convert a map in our format to the toolbox format.
[map, bbox, cost_map] = obstacle_map_to_toolbox(map_ros);

%% Create cost function derivatives used by CHOMP
[ cost_map_x, cost_map_y] = get_cost_map_derivatives( cost_map );

%% Create Initial Guess
n = 100; %How many waypoints
xi = [linspace(start(1),goal(1),n)' linspace(start(2),goal(2),n)'];

%% Create surrogate cost function used by CHOMP
w_obs = 100;
lambda = 1;

[ A, b, c ] = smooth_matrices(start, goal, n-2);
f_smooth = @(xi) cost_smooth( xi, A, b, c);
grad_smooth = @(xi) fsmooth(xi, A, b);  

c_scalar_obs_fn = @(xi) cost_obs( xi, cost_map, start );
grad_fobs = @(xi) gradient_obs( xi, cost_map, cost_map_x, cost_map_y, start );

c_final = @(xi, xi_der) lambda*f_smooth(xi) + w_obs*c_scalar_obs_fn(xi);
grad_final = @(xi, xi_der) lambda*grad_smooth(xi) + w_obs*grad_fobs(xi);

%% Set optimization options
options.eta = 1000;
options.min_cost_improv_frac = 1e-5;
options.M = A;
%options.M = eye(size(A));
options.decrease_wt = 1;
options.progress = 1;

%% Set stopping conditions
options.max_iter = 1000;
options.min_iter = 10;
options.min_cost_improv_frac = 1e-5;
options.max_time = 10.0;

%% Visualize
options.visualize = 0;

%% Setup visualizations
axis(reshape(bbox, 1, []));
hold on;
if (visualize)
  visualize_map(map);
end

%% Do CHOMP
[final_path, final_cost, num_iter] = covariant_gradient_descent( xi, c_final, grad_final, options );

%% Check path
found_path = ~isempty(final_path);
fprintf('Found path: %d \n', found_path);
if (found_path)
    in_collision = cost_fn_map_coll_dense(final_path, map);
    fprintf('Is solution in collision: %d \n', in_collision);
    if (~in_collision)
        fprintf('Length of solution: %f\n', traj_length(final_path));
    end
end

if (visualize)
  plot(final_path(:, 1), final_path(:, 2), 'LineWidth', 2);
end

%% Create metrics
metrics = create_metrics_struct;
metrics.success = ~in_collision;
metrics.length = traj_length(final_path);
metrics.final_cost = final_cost;
metrics.initial_cost = c_final(xi(2:end-1, :), []);
metrics.num_iter = num_iter;
end

