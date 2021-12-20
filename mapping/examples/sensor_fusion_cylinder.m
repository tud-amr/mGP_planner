% an example to show sensor data sequential fusion
close all
clear all
clear 
clc 


%% parameters
% map environments
data_mesh = load('cylinder_mesh.mat');
TR = data_mesh.TR;
num_faces = size(TR.ConnectivityList, 1);
num_vertices = size(TR.Points, 1);
F_normal = faceNormal(TR);
F_center = incenter(TR);
F_points = zeros(num_faces, 3, 3);
for iFace = 1 : num_faces
    F_points(iFace, :, 1) = TR.Points(TR.ConnectivityList(iFace, 1), :);    % 1x3
    F_points(iFace, :, 2) = TR.Points(TR.ConnectivityList(iFace, 2), :);
    F_points(iFace, :, 3) = TR.Points(TR.ConnectivityList(iFace, 3), :);
end
% sensor parameter, fixed
sensor_parameters.cam_roll = 0;
sensor_parameters.cam_pitch = deg2rad(15);
sensor_parameters.cam_yaw = 0;
sensor_parameters.fov_x = deg2rad(60);
sensor_parameters.fov_y = deg2rad(60);
sensor_parameters.fov_range_min = 1;
sensor_parameters.fov_range_max = 8;
sensor_parameters.incidence_range_min = cos(deg2rad(70));
sensor_parameters.sensor_coeff_A = 0.05;
sensor_parameters.sensor_coeff_B = 0.2;
% map parameters
map_parameters.sigma_f = 1.3;
map_parameters.l = 0.3;
map_parameters.num_faces = num_faces;
map_parameters.F_center = F_center;
map_parameters.F_normal = F_normal;
map_parameters.F_points = F_points;
% matlab parameters
matlab_parameters.visualize = 1;


%% list of viewpoints where measurements are taken
% include position and mav yaw
viewpoints_list = [5, -5, 15, deg2rad(90);
                   12,-3, 12, deg2rad(140)];
viewpoints_list(:,1:2) = viewpoints_list(:,1:2) - 6;


%% ground truth map
data_map = load('map_3D.mat');
map_3D = data_map.ground_truth_map;
% dimensions [m]
dim_x_env = 40;
dim_y_env = 40;
dim_z_env = 40;
resolution = 0.5;
dim_x = dim_x_env / resolution;
dim_y = dim_y_env / resolution;
dim_z = dim_z_env / resolution;
F_value = zeros(num_faces, 1);
for iF = 1 : num_faces
    center_iF = F_center(iF, :);
    idx_iF(1:2) = round(center_iF(1:2) / resolution) + 40;
    idx_iF(3) = round(center_iF(3) / resolution) + 20;
    F_value(iF) = map_3D(idx_iF(1), idx_iF(2), idx_iF(3));
end
ground_true_faces_map = F_value;


%% prediction map
faces_map.m = zeros(num_faces, 1);
faces_map.P = zeros(num_faces, num_faces);


%% prior map
faces_map.m = 0.5.*ones(size(faces_map.m));
for i = 1 : num_faces
    for j = i : num_faces
        d_ij = norm(F_center(i,:)-F_center(j,:));
        k_ij = cov_materniso_3(d_ij/10, map_parameters.sigma_f, map_parameters.l);
        faces_map.P(i, j) = k_ij;
        faces_map.P(j, i) = k_ij;
    end
end
% faces_map.P = eye(num_faces);       % for debugging
P_prior = diag(faces_map.P);


%% sequential measurements and data fusion
num_measurements = size(viewpoints_list, 1);
if matlab_parameters.visualize
    
    figure;
    
    % ground truth map
    subplot(2, num_measurements+2, 1);
    hold on;
    axis([-9 9 -9 9 0 25]);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Ground truth map')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), F_value, 'EdgeAlpha', 0);
    caxis([0, 1]);
    
    % prior mean
    subplot(2, num_measurements+2, 2);
    hold on;
    axis([-9 9 -9 9 0 25]);
    xlabel('x [m]');
    ylabel('y [m]');
    zlabel('z [m]');
    title('Mean - prior')
    daspect([1 1 1]);
    view(3);
    trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
        TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
    caxis([0, 1]);
    
    % prior variance
    subplot(2, num_measurements+2, num_measurements+4);
    hold on;
    axis([-9 9 -9 9 0 25]);
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

P_trace_prev = trace(faces_map.P);
for i = 1 : num_measurements
    
%     pause;
    
    viewpoint = viewpoints_list(i, :);
    faces_map = take_measurement_at_viewpoint(viewpoint, faces_map, ...
        ground_true_faces_map, map_parameters, sensor_parameters);
    P_post = diag(faces_map.P);
    
    if matlab_parameters.visualize
        
        % mean
        subplot(2, num_measurements+2, i+2);
        hold on;
        axis([-9 9 -9 9 0 25]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        title(['Mean - after ', num2str(i), ' meas.'])
        daspect([1 1 1]);
        view(3);
        trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
            TR.Points(:,3), faces_map.m, 'EdgeAlpha', 0);
        caxis([0 1]);
        if (i == num_measurements)
%             colorbar;
        end
        
        % variance
        subplot(2, num_measurements+2, num_measurements+4+i);
        hold on;
        axis([-9 9 -9 9 0 25]);
        xlabel('x [m]');
        ylabel('y [m]');
        zlabel('z [m]');
        title(['Var. Trace = ', num2str(trace(faces_map.P), 5)])
        daspect([1 1 1]);
        view(3);
        trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
            TR.Points(:,3), P_post, 'EdgeAlpha', 0);
        caxis([0 var_max]);
        if (i == num_measurements)
%             colorbar;
        end
        
    end
    
    disp(['Measurement No. ', num2str(i), ': ', num2str(viewpoint)]);
    disp(['Trace of P: ', num2str(trace(faces_map.P))]);
    disp(['Diff. in trace of P: ', num2str(trace(P_trace_prev - trace(faces_map.P)))]);
    
    P_trace_prev = trace(faces_map.P);
    
end

