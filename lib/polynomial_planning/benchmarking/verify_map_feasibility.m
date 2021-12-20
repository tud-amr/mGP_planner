function [feasible] = verify_map_feasibility(map_ros, start, goal, visualize)


% Convert a map in our format to the toolbox format.
[map, bbox, ~] = obstacle_map_to_toolbox(map_ros);

%% Get global search options
options = global_search_options(start, goal, bbox); % get default options

%% Setup cost function
options.c = @(v1, v2) cost_fn_map_coll_traj(v1.state, v2.state, map, inf);

%% Setup heuristic
heuristic_inflation = 10;
options.h_hat = @(v) heuristic_inflation*pdist2(cell2mat({v.state}'), goal);

%% Setup Implicit Graph
resolution = 40;
options.sampler = @(g_t) sampling_lattice2D_static(g_t, resolution, start, goal, bbox);
options.succ_func = @(query, V, S, total_size) succ_func_lattice2D_4conn_static( query, V, S, total_size, resolution, bbox);

%% Set stopping conditions
options.max_batches = 1;
options.max_iter = inf;
options.max_time = 60.0;

%% Do you want to visualize planner progress (it will take longer)
options.visualize = 0;

%% Setup visualizations
figure;
axis(reshape(bbox, 1, []));
hold on;
visualize_map(map);

%% Call Planner
[final_path] = batch_sample_planner( start, goal, options );

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

feasible = found_path && ~in_collision;

if (visualize)
  plot(final_path(:, 1), final_path(:, 2))
end

end

