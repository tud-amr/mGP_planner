% a plot planning example
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


%% Planning results
if model_name == 'cylinder'
    results_name = 'cylinder_cmaes_kernel_5_metrics'; 
elseif model_name == 'boeing747'
    results_name = 'boeing747_cmaes_kernel_2_metrics';
else
    error('Results not found!')
end
load([results_name, '.mat']);

%% Ground truth and initial map
dim_x_env = map_parameters.dim_x_env;
dim_y_env = map_parameters.dim_y_env;
dim_z_env = map_parameters.dim_z_env;
dim_xyz_plot = [dim_x_env, dim_y_env, 0, dim_z_env(2)];
% dim_xyz_plot = [dim_x_env, dim_y_env, dim_z_env];
ground_truth_faces_map = create_ground_truth_map(map_parameters);
faces_map = create_initial_map(map_parameters);
P_prior = diag(faces_map.P);

if (matlab_parameters.visualize_map)
    
    figure;
    
    subplot(2, 3, 1)
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
    
    subplot(2, 3, 2)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Mean - inital')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
    caxis([0, 1]);
    colormap jet
    
    subplot(2, 3, 5)
    hold on;
    axis(dim_xyz_plot);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title(['Var. - initial. Trace = ', num2str(trace(faces_map.P), 5)])
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), P_prior, 'EdgeAlpha', 0);
    var_max = max(P_prior);
    caxis([0 var_max]);
 
end

%% Take first measurement
% viewpoint_init = [-7.0711   -7.0711    4.0000    0.7854]; %[10, 0, 4, -pi]
viewpoint_init = [-4   0    0    0];
% comment if not taking a first measurement
faces_map = take_measurement_at_viewpoint(viewpoint_init, faces_map, ...
        ground_truth_faces_map, map_parameters, sensor_parameters);
P_post = diag(faces_map.P);
P_trace_init = trace(faces_map.P);
P_prior = P_post;

% if (matlab_parameters.visualize_map)
% 
%     subplot(2, 4, 3)
%     hold on;
%     axis(dim_xyz_plot);
%     xlabel('x [m]');
%     ylabel('y [m]');
%     zlabel('z [m]');
%     title('Mean - init ')
%     daspect([1 1 1]);
%     view(3);
%     trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
%         TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
%     caxis([0 1]);
%     colormap jet
%     
%     subplot(2, 4, 7)
%     hold on;
%     axis(dim_xyz_plot);
%     xlabel('x [m]');
%     ylabel('y [m]');
%     zlabel('z [m]');
%     title(['Var. - init Trace = ', num2str(trace(faces_map.P), 5)])
%     daspect([1 1 1]);
%     view(3);
%     trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
%         TR.Points(:,3), P_post, 'EdgeAlpha', 0);
%     caxis([0 var_max]);
%     
% end

%% Planning results
if (matlab_parameters.visualize_map)
    
    subplot(2, 3, 3)
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
    
    subplot(2, 3, 6)
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
    axis(dim_xyz_plot);
    plot_path_viewpoints(ax_path, num_path_segments, metrics.path_travelled, ...
        metrics.trajectory_travelled, metrics.viewpoints_meas);

    % camera fov
    if (matlab_parameters.visualize_cam) % matlab_parameters.visualize_cam, true
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

%% Quantantive results
figure;
plot_metrics(metrics);
