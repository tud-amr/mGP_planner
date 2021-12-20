% a planning example
close all
clear all
clear 
clc 

% Random number generator
matlab_parameters.seed_num = 3;
rng(matlab_parameters.seed_num, 'twister');


%% Environment
model_name = 'cylinder'; % cylinder, boeing747
model.name = model_name;
% mesh
data_mesh = load([model_name, '_mesh.mat']);
model.TR = data_mesh.TR;
model.valid_faces = data_mesh.valid_faces;
TR = data_mesh.TR;
% occupancy
data_occupancy = load([model_name, '_map_occupancy']);
model.occupancy = data_occupancy.occupancy; 
% esdf
data_esdf = load([model_name, '_map_esdf']);
model.esdf = data_esdf.esdf; 
% true temperature field
data_temperature_field = load([model_name, '_temperature_field']);
model.temperature_field = data_temperature_field.F_value;


%% Parameters
[map_parameters, sensor_parameters, planning_parameters, optimization_parameters, ...
    matlab_parameters] = load_parameteres(model);


%% Ground truth and initial map
dim_x_env = map_parameters.dim_x_env;
dim_y_env = map_parameters.dim_y_env;
dim_z_env = map_parameters.dim_z_env;
dim_xyz_plot = [dim_x_env, dim_y_env, 0, dim_z_env(2)];
% dim_xyz_plot = [dim_x_env, dim_y_env, dim_z_env];
% dim_xyz_plot = [-9 9 -9 9 0 25];
% dim_xyz_plot = [-8 80 -40 40 -4 16];
ground_truth_faces_map = create_ground_truth_map(map_parameters);
faces_map = create_initial_map(map_parameters);
P_prior = diag(faces_map.P);
if (matlab_parameters.visualize_map)
    
    figure;
    
    subplot(2, 4, 1)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Ground truth map')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), ground_truth_faces_map, 'EdgeAlpha', 0);
    caxis([0, 1]);
    colormap jet
    
    subplot(2, 4, 2)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Mean - prior')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
    caxis([0, 1]);
    colormap jet
    
    subplot(2, 4, 6)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(['Var. - prior. Trace = ', num2str(trace(faces_map.P), 5)])
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), P_prior, 'EdgeAlpha', 0);
    var_max = max(P_prior);
    caxis([0 var_max]);
 
end


%% Take first measurement
viewpoint_init = [-7.0711   -7.0711    4.0000    0.7854]; %[10, 0, 4, -pi]
% viewpoint_init = [-4   0    0    0];
faces_map = take_measurement_at_viewpoint(viewpoint_init, faces_map, ...
        ground_truth_faces_map, map_parameters, sensor_parameters);
P_post = diag(faces_map.P);
P_trace_init = trace(faces_map.P);
P_prior = P_post;

if (matlab_parameters.visualize_map)

    subplot(2, 4, 3)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Mean - init ')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
    caxis([0 1]);
    colormap jet
    
    subplot(2, 4, 7)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(['Var. - init Trace = ', num2str(trace(faces_map.P), 5)])
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), P_post, 'EdgeAlpha', 0);
    caxis([0 var_max]);
    
end


%% Lattice viewpoints
data_lattice = load([model_name, '_lattice_viewpoints.mat']);
lattice_viewpoints = data_lattice.lattice_viewpoints;
num_lattice_viewpoints = size(lattice_viewpoints, 1);


%% Planning:
tic

%% Step 1: Grid search on the lattice viewpoints
P_trace_prev = P_trace_init;
viewpoint_prev = viewpoint_init;
faces_map_plan = faces_map;
path = search_lattice_viewpoints(viewpoint_prev, lattice_viewpoints, ...
    faces_map, map_parameters, sensor_parameters, planning_parameters);
obj = compute_objective_inspect(path, faces_map, map_parameters, sensor_parameters, ...
    planning_parameters, optimization_parameters);
disp(['Objective before optimization: ', num2str(obj)]);

%% STEP 2. CMA-ES optimization, only optimize position now.
path_optimized = optimize_with_cmaes_inspect(path, faces_map, map_parameters, ...
    sensor_parameters, planning_parameters, optimization_parameters);

computation_time = toc

%% Plan Execution %%
% Create polynomial path through the control points.
trajectory = plan_path_waypoints(path_optimized(:,1:3), ...
        planning_parameters.max_vel, planning_parameters.max_acc);
% Find best yaw for each control point if not optimizing
if (optimization_parameters.opt_yaw)
    control_yaws = path_optimized(:,4);
else
    control_yaws = zeros(size(path_optimized, 1), 1);
    for i = 1 : size(path_optimized,1)
        control_yaws(i) = get_best_yaw(path_optimized(i,1:3), map_parameters);
    end
end
% Also create the yaw trajectory
segment_time = zeros(trajectory.num_elements, 1);
for i = 2 : trajectory.num_elements
    segment_time(i) = trajectory.segments(i-1).time;
end
yaw_trajectory = plan_yaw_waypoints(control_yaws, segment_time);

% Sample trajectory to find locations to take measurements at.
[t, measurement_points, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);
[~, measurement_yaws, ~, ~] = sample_trajectory(yaw_trajectory, ...
    1/planning_parameters.measurement_frequency);

% Alternatively, spercify a yaw to the measurement point
if planning_parameters.plan_yaw == 0
    for i = 1 : size(measurement_points,1)
        measurement_yaws(i) = get_best_yaw(measurement_points(i,1:3), map_parameters);
    end
end

% Find the corresponding yaw
num_points_meas = size(measurement_points,1);
viewpoints_meas = [measurement_points, measurement_yaws];

% Take measurements along path.
for i = 2:size(measurement_points,1)
    faces_map = take_measurement_at_viewpoint(viewpoints_meas(i,:), faces_map, ...
            ground_truth_faces_map, map_parameters, sensor_parameters);
end

disp(['Trace after execution: ', num2str(trace(faces_map.P))]);
disp(['Time after execution: ', num2str(get_trajectory_total_time(trajectory))]);
gain = P_trace_init - trace(faces_map.P);
if (strcmp(planning_parameters.obj, 'rate'))
    cost = max(get_trajectory_total_time(trajectory), 1/planning_parameters.measurement_frequency);
    disp(['Objective after optimization: ', num2str(-gain/cost)]);
elseif (strcmp(planning_parameters.obj, 'exponential'))
    cost = get_trajectory_total_time(trajectory);
    disp(['Objective after optimization: ', num2str(-gain*exp(-planning_parameters.lambda*cost))]);
end
P_post = diag(faces_map.P);

if (matlab_parameters.visualize_map)
    
    subplot(2, 4, 4)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Mean - final ')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
    caxis([0 1]);
    
    subplot(2, 4, 8)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(['Var. - final Trace = ', num2str(trace(faces_map.P), 5)])
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), P_post, 'EdgeAlpha', 0);
    caxis([0 var_max]);
    
end

if (matlab_parameters.visualize_path)
    
    fig_path = figure;
    hold on;
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    ax_path = fig_path.CurrentAxes;
    daspect(ax_path, [1 1 1]);
    view(ax_path, 3);
    
    % path and viewpoints
    axis([map_parameters.dim_x_env map_parameters.dim_y_env ...
        0 map_parameters.dim_z_env(2)]);
    plot_path_viewpoints(ax_path, 1, path_optimized, trajectory, viewpoints_meas);
    
    dt = 0.5;
    [t, p] = sample_trajectory(trajectory, dt);
    [t_yaw, yaw] = sample_trajectory(yaw_trajectory, dt);
    plot_trajectory_cline(t, p, yaw);
    
    
    % mesh object
    h_mesh = trimesh(TR);
    h_mesh.FaceColor = 'w';
    h_mesh.FaceAlpha = 1;
    h_mesh.EdgeColor = 'c';
    h_mesh.LineWidth = 0.5;
    h_mesh.LineStyle = '-';
    
    % camera fov
    for i = 1 : num_points_meas
%         pause;
        cam_pos = viewpoints_meas(i, 1:3)';
        cam_roll = sensor_parameters.cam_roll;
        cam_pitch = sensor_parameters.cam_pitch;
        cam_yaw = sensor_parameters.cam_yaw + viewpoints_meas(i,4);
        plot_camera_fov(ax_path, cam_pos, cam_roll, cam_pitch, cam_yaw, ...
            sensor_parameters.fov_x, sensor_parameters.fov_y, ...
            sensor_parameters.fov_range_max, 'r');
        [F_visible, faces_visible] = get_visible_faces(map_parameters.num_faces, ...
            map_parameters.F_points, map_parameters.F_center, ...
            map_parameters.F_normal, cam_pos, cam_roll, cam_pitch, cam_yaw, sensor_parameters);
        for iFace = 1 : map_parameters.num_faces
            if F_visible(iFace) == 1
                patch(ax_path, 'XData', map_parameters.F_points(iFace, 1, :), ...
                      'YData', map_parameters.F_points(iFace, 2, :), ...
                      'ZData', map_parameters.F_points(iFace, 3, :), ...
                      'FaceColor', 'b', ... 
                      'FaceAlpha', 0.5, ...
                      'EdgeColor', 'b');
            end
        end
    end
    
end
