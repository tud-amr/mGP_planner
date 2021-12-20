% make animation of the inspection mission
close all 
clear all
clear 
clc 

%% Load planning results (only contains information metrics.mat)
results_name = 'cylinder_cmaes_kernel_5_metrics'; 
% results_name = 'boeing747_cmaes_kernel_2_metrics';
load([results_name, '.mat']);
viewpoint_init = [-7.0711   -7.0711    4.0000    0.7854];
% viewpoint_init = [-4   0    0    0];


%% Insepection setups
% model
model_name = 'cylinder';        % cylinder, boeing747
model.name = model_name;
radius = 1.2;
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
% parameters
[map_parameters, sensor_parameters, planning_parameters, optimization_parameters, ...
    matlab_parameters] = load_parameteres(model);


%% Ground truth
ground_truth_faces_map = create_ground_truth_map(map_parameters);
faces_map = create_initial_map(map_parameters);
P_prior = diag(faces_map.P);


%% Environment and plot dimensions
dim_x_env = map_parameters.dim_x_env;
dim_y_env = map_parameters.dim_y_env;
dim_z_env = map_parameters.dim_z_env;
switch model.name
    case 'cylinder'
        dim_xyz_map = [-8 8 -8 8 0 22];
        dim_xyz_path = [-14 14 -14 14 0 26];
    case 'boeing747'
        dim_xyz_map = [-8 80 -40 40 -4 16];
        dim_xyz_path = [-8 80 -40 40 -4 16];
    otherwise
        error('Model not found!')
end


%% Plot handles and initial plot
%% ground truth
fig_map_truth = figure;
hold on;
axis(dim_xyz_map);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_truth = fig_map_truth.CurrentAxes;
daspect([1 1 1]);
view(3);
title('Ground truth map');
set(gcf, 'Position', [100, 600, 400, 400]);
set(gca, 'FontSize', 12);
h_map_true = trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), ground_truth_faces_map, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet
hold off
%% map mean
fig_map_mean = figure;
hold on;
axis(dim_xyz_map);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_map_mean = fig_map_mean.CurrentAxes;
daspect([1 1 1]);
view(3);
title('Map mean');
set(gcf, 'Position', [503, 600, 400, 400]);
set(gca, 'FontSize', 12);
h_map_mean = trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet
hold off
%% map variance
fig_map_var = figure;
hold on;
axis(dim_xyz_map);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_map_var = fig_map_var.CurrentAxes;
daspect([1 1 1]);
view(3);
title('Map variance');
set(gcf, 'Position', [915, 600, 400, 400]);
set(gca, 'FontSize', 12);
h_map_var = trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), P_prior, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet
hold off
%% inspection path
% 3D model
fig_path = figure;
hold on;
axis(dim_xyz_path);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_path = fig_path.CurrentAxes;
daspect([1 1 1]);
view(3);
title('Inspection');
set(gcf, 'Position', [1400, 500, 500, 500]);
set(gca, 'FontSize', 14);
h_path_object = trimesh(TR, 'FaceColor', 'w', 'FaceAlpha', 1, ...
    'EdgeColor', 'c', 'LineWidth', 0.5);
% control point
h_path_control_points = scatter3(ax_path, viewpoint_init(1), viewpoint_init(2), ...
    viewpoint_init(3), 120, 'xk', 'MarkerFaceColor',[0 .75 .75]);
% trajectory
h_path_traj = cline(viewpoint_init(1), viewpoint_init(2), ...
    viewpoint_init(3), 0);
% measurement viewpoints
h_path_meas_points = scatter3(ax_path, viewpoint_init(1), viewpoint_init(2), ...
    viewpoint_init(3),50, 'b', 'filled');
% camera fov
h_cam_fov = plot_camera_fov(ax_path, viewpoint_init(1:3)', ...
    sensor_parameters.cam_roll, sensor_parameters.cam_pitch, ...
    viewpoint_init(4), ...
    sensor_parameters.fov_x, sensor_parameters.fov_y, ...
    sensor_parameters.fov_range_max, 'r');
% quadrotor
R_quad = rotXYZ([sensor_parameters.cam_roll, sensor_parameters.cam_pitch, ...
    viewpoint_init(4)]);
h_pos_quad = plot_quadrotor_3D(ax_path, viewpoint_init(1:3)', R_quad, radius, ...
        'r','FaceColor', 'r', 'FaceAlpha', 0.1, ...
        'EdgeColor', 'r', 'EdgeAlpha', 0.8);
%% uncertainty reduction
fig_trP = figure;
hold on;
box on;
grid on;
axis([0 planning_parameters.time_budget 0 7000])
xlabel('Time (s)');
ylabel('Trace of P');
title('Uncertainty reduction');
ax_trP = fig_trP.CurrentAxes;
set(gcf, 'Position', [100, 50, 400, 300]);
set(ax_trP, 'FontSize', 12);
h_trP = plot(ax_trP, metrics.times(1), metrics.P_traces(1), 'LineWidth', 2.5, ...
    'LineStyle', '-', 'Color', [0,102,162]/256);
%% mapping error
fig_RMSE = figure;
hold on;
box on;
grid on;
axis([0 planning_parameters.time_budget 0 0.35])
xlabel('Time (s)');
ylabel('RMSE');
title('Mapping error');
ax_RMSE = fig_RMSE.CurrentAxes;
set(gcf, 'Position', [600, 50, 400, 300]);
set(ax_RMSE, 'FontSize', 12);
h_RMSE = plot(ax_RMSE, metrics.times(1), metrics.rmses(1), 'LineWidth', 2.5, ...
    'LineStyle', '-', 'Color', [235,114,70]/256);


%% Results processing
dt = 0.1;
num_ctl_points = planning_parameters.control_points;
num_traj_segm = size(metrics.trajectory_travelled, 1);
path_travelled = zeros(num_ctl_points, 4, num_traj_segm);
traj_travelled = cell(num_traj_segm,1);
viewpoints_meas = cell(num_traj_segm,1);
disp('Press Enter to start!');
pause;
for i = 1 : num_traj_segm
    % path
    path_travelled(:, 1:3, i) = metrics.path_travelled(1+4*(i-1):4*i, 1:3);
    for j = 1 : num_ctl_points
        path_travelled(j, 4, i) = get_best_yaw(path_travelled(j, 1:3, i), map_parameters);
    end
    % pos trajectory
    [t_poly, p_poly] = sample_trajectory(metrics.trajectory_travelled(i), dt);
    traj_poly = p_poly;
    % yaw trajectory
%     segment_time = zeros(metrics.trajectory_travelled(i).num_elements, 1);
%     for j = 2 : metrics.trajectory_travelled(i).num_elements
%         segment_time(j) = metrics.trajectory_travelled(i).segments(j-1).time;
%     end
%     yaw_trajectory = plan_yaw_waypoints(path_travelled(:, 4, i), segment_time);
%     [t_yaw, yaw] = sample_trajectory(yaw_trajectory, dt);
%     traj_poly(:, 4) = yaw;
%     traj_travelled{i} = traj_poly;
%     % measurement
%     [t, measurement_points, ~, ~] = ...
%         sample_trajectory(metrics.trajectory_travelled(i), ...
%         1/planning_parameters.measurement_frequency);
%     [~, measurement_yaws, ~, ~] = sample_trajectory(yaw_trajectory, ...
%         1/planning_parameters.measurement_frequency);
%     viewpoints_meas{i} = [measurement_points, measurement_yaws];
    % an alternative way
    for j = 1 : length(traj_poly)
        traj_poly(j, 4) = get_best_yaw(traj_poly(j, 1:3), map_parameters);
    end
    traj_travelled{i} = traj_poly;
end


%% Animation
time_now = 0;
time_measure = -5;
idx_measure = 0;
for i = 1 : num_traj_segm
    % update control points
%     set(h_path_control_points, 'XData', path_travelled(:, 1, i), ...
%         'YData', path_travelled(:, 2, i), 'ZData', path_travelled(:, 1, i));
    % update planned trajectory
    plot3(ax_path, traj_travelled{i}(:,1), traj_travelled{i}(:,2), ...
        traj_travelled{i}(:, 3), 'LineWidth', 2, 'Color', [0,166,214]/256, ...
        'LineStyle', '-');
    % move the quadrotor
    num_pos = length(traj_travelled{i});
    for j = 1 : num_pos
        if time_now >= planning_parameters.time_budget
            break;
        end
        % time
        time_now = time_now + dt;
        % update quad pos
        delete(h_pos_quad{1}); delete(h_pos_quad{2}); delete(h_pos_quad{3});
        pos_quad = traj_travelled{i}(j, 1:3)';
        R_quad = rotXYZ([sensor_parameters.cam_roll, sensor_parameters.cam_pitch, ...
            traj_travelled{i}(j, 4)]);
        h_pos_quad = plot_quadrotor_3D(ax_path, pos_quad, R_quad, radius, ...
                'r','FaceColor', 'r', 'FaceAlpha', 0.1, ...
                'EdgeColor', 'r', 'EdgeAlpha', 0.8);
        % add orientation
        yaw_quad = traj_travelled{i}(j, 4);
        if abs(mod(time_now, 0.5)) <= dt
            u = 2*cos(yaw_quad);
            v = 2*sin(yaw_quad);
            w = 0;
            quiver3(ax_path, pos_quad(1), pos_quad(2), pos_quad(3), u, v, w, ...
                'Color', [255,153,153]/256, 'LineWidth', 1.0, 'MaxHeadSize', 0.6);
        end
        % detect if reaching a measurement viewpoint
        if sum(abs(time_now - metrics.times) <= dt) > 0 && time_now - time_measure > 0.12
            time_measure = time_now;
            idx_measure = idx_measure + 1;
            if idx_measure > length(metrics.viewpoints_meas)
                idx_measure = length(metrics.viewpoints_meas);
            end
            % add a measurement viewpoint
            scatter3(ax_path, pos_quad(1), pos_quad(2), ...
                pos_quad(3),50, 'b', 'filled');
            % update fov
            delete(h_cam_fov);
            h_cam_fov = plot_camera_fov(ax_path, pos_quad, ...
                sensor_parameters.cam_roll, sensor_parameters.cam_pitch, ...
                yaw_quad, sensor_parameters.fov_x, sensor_parameters.fov_y, ...
                sensor_parameters.fov_range_max, 'r');
            % update map mean
            figure(fig_map_mean);       % make the figure active
            delete(h_map_mean);
            hold(ax_map_mean, 'on');
            h_map_mean = trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
                TR.Points(:,3), metrics.faces_map_m(idx_measure, :), 'EdgeAlpha', 0);
            hold(ax_map_mean, 'off');
            % update map variance
            figure(fig_map_var);
            delete(h_map_var);
            hold(ax_map_var, 'on');
            h_map_var = trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
                TR.Points(:,3), metrics.faces_map_P_diag(idx_measure, :), 'EdgeAlpha', 0);
            hold(ax_map_var, 'off');
            % update P trace
            set(h_trP, 'XData', metrics.times(1:idx_measure), 'YData', metrics.P_traces(1:idx_measure));
            % update RMSE
            set(h_RMSE, 'XData', metrics.times(1:idx_measure), 'YData', metrics.rmses(1:idx_measure));
            % keep a pause
            pause(0.1);
        end
        % update the figure
        drawnow limitrate
        pause(0.02);
    end
end

