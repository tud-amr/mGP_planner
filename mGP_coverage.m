% a planning example
close all
clear all
clear 
clc 

root_folder = pwd;

% Random number generator
% matlab_parameters.seed_num = 3;
% rng(matlab_parameters.seed_num, 'twister');


%% Environment
model_name = 'cylinder';
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
ground_truth_faces_map = create_ground_truth_map(map_parameters);
faces_map = create_initial_map(map_parameters);
P_prior = diag(faces_map.P);

dim_xyz_plot = [-9 9 -9 9 0 25];
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
% comment if not taking a first measurement
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


%% Coverage path
data_coverage_path = load([model_name, '_coverage_path.mat']);
coverage_path = data_coverage_path.coverage_path;


%% Execution
metrics = initialize_metrics_inspect();

% Create polynomial path through the control points.
trajectory = plan_path_waypoints(coverage_path(:,1:3), ...
        planning_parameters.max_vel, planning_parameters.max_acc);
    
% Read control yaws
control_yaws = coverage_path(:, 4);

% Also create the yaw trajectory
segment_time = zeros(trajectory.num_elements, 1);
for i = 2 : trajectory.num_elements
    segment_time(i) = trajectory.segments(i-1).time;
end
yaw_trajectory = plan_yaw_waypoints(control_yaws, segment_time);

% Sample trajectory to find locations to take measurements at.
[times_meas, points_meas, ~, ~] = ...
    sample_trajectory(trajectory, 1/planning_parameters.measurement_frequency);
[~, yaws_meas, ~, ~] = sample_trajectory(yaw_trajectory, ...
    1/planning_parameters.measurement_frequency);
trajectory_time = get_trajectory_total_time(trajectory);

% Alternatively, spercify a yaw to the measurement point
if planning_parameters.plan_yaw == 0
    for i = 1 : size(points_meas,1)
        yaws_meas(i) = get_best_yaw(points_meas(i,1:3), map_parameters);
    end
end

% Remove the viewpoints beyond budget
idx_in_budget = find(times_meas <= planning_parameters.time_budget);
if length(idx_in_budget) < length(times_meas)
    trajectory_time = planning_parameters.time_budget;
end
times_meas = times_meas(idx_in_budget);
points_meas = points_meas(idx_in_budget,:);
yaws_meas = yaws_meas(idx_in_budget,:);

% Combine the viewpoints
num_points_meas = size(points_meas,1);
viewpoints_meas = [points_meas, yaws_meas];

% Take measurements along path.
for i = 1:num_points_meas
    faces_map = take_measurement_at_viewpoint(viewpoints_meas(i,:), faces_map, ...
            ground_truth_faces_map, map_parameters, sensor_parameters);
    metrics.faces_map_m = [metrics.faces_map_m; faces_map.m'];
    metrics.faces_map_P_diag = [metrics.faces_map_P_diag; diag(faces_map.P)'];
    metrics.P_traces = [metrics.P_traces; trace(faces_map.P)];
    metrics.rmses = [metrics.rmses; compute_rmse(faces_map.m(map_parameters.valid_faces), ...
        ground_truth_faces_map(map_parameters.valid_faces))];
    metrics.wrmses = [metrics.wrmses; compute_wrmse(faces_map.m(map_parameters.valid_faces), ...
        ground_truth_faces_map(map_parameters.valid_faces))];
    metrics.mlls = [metrics.mlls; compute_mll(faces_map, ground_truth_faces_map)];
    metrics.wmlls = [metrics.wmlls; compute_wmll(faces_map, ground_truth_faces_map)];
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

metrics.viewpoints_meas = [metrics.viewpoints_meas; viewpoints_meas];
metrics.times = [metrics.times; times_meas'];
metrics.path_travelled = [metrics.path_travelled; coverage_path];
metrics.trajectory_travelled = [metrics.trajectory_travelled; trajectory];

disp(['Time elapsed: ', num2str(times_meas)]);


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
        TR.Points(:,3), metrics.faces_map_m(end,:)', 'EdgeAlpha', 0);
    caxis([0 1]);
    colormap jet
    
    subplot(2, 4, 8)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(['Var. - final Trace = ', num2str(metrics.P_traces(end,:), 5)])
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), metrics.faces_map_P_diag(end,:)', 'EdgeAlpha', 0);
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
    
    % mesh object
    h_mesh = trimesh(TR);
    h_mesh.FaceColor = 'w';
    h_mesh.FaceAlpha = 1;
    h_mesh.EdgeColor = 'c';
    h_mesh.LineWidth = 0.5;
    h_mesh.LineStyle = '-';
    
    num_path_segments = size(metrics.trajectory_travelled, 1);
    % path and viewpoints
    axis([map_parameters.dim_x_env map_parameters.dim_y_env ...
        0 map_parameters.dim_z_env(2)]);
    plot_path_viewpoints(ax_path, num_path_segments, metrics.path_travelled, ...
        metrics.trajectory_travelled, metrics.viewpoints_meas);

    % camera fov
    if (matlab_parameters.visualize_cam)
        for i = 1 : size(metrics.viewpoints_meas, 1)
%             pause;
            cam_pos = metrics.viewpoints_meas(i, 1:3)';
            cam_roll = sensor_parameters.cam_roll;
            cam_pitch = sensor_parameters.cam_pitch;
            cam_yaw = sensor_parameters.cam_yaw + metrics.viewpoints_meas(i,4);
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
    
end

figure;
plot_metrics(metrics);

save([root_folder, '/logs/cylinder/', model_name, '_coverage_', 'kernel_', ...
    num2str(map_parameters.kernel_choice), '_metrics.mat'], 'metrics');
