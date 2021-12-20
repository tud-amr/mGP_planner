% close all
clear all
clear 
clc 

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

%% Load data
% triangulation properties
num_faces = size(TR.ConnectivityList, 1);
num_vertices = size(TR.Points, 1);
F_normal = faceNormal(TR);
F_center = incenter(TR);
F_points = zeros(num_faces, 3, 3);
for iFace = 1 : num_faces
    F_points(iFace, :, 1) = TR.Points(TR.ConnectivityList(iFace, 1), :);
    F_points(iFace, :, 2) = TR.Points(TR.ConnectivityList(iFace, 2), :);
    F_points(iFace, :, 3) = TR.Points(TR.ConnectivityList(iFace, 3), :);
end

%% Parameters
[map_parameters, sensor_parameters, planning_parameters, optimization_parameters, ...
    matlab_parameters] = load_parameteres(model);

%% camera state
viewpoints = [  0         0         4.0000    0.7854;
                0.6098   -4.0791   14.0478    1.0797;
                8.3115   -2.0126   24.8889    1.8517;
                17.8547    8.3258   20.0581   -2.9479;
                16.2890   14.3044    9.4139   -2.4625;
                9.5461   19.6234   14.6199   -1.8254];
viewpoints(:,1:2) = viewpoints(:,1:2) - 6;
i = 2;
cam_pos = viewpoints(i, 1:3)';
cam_yaw = sensor_parameters.cam_yaw + viewpoints(i, 4);
cam_roll = sensor_parameters.cam_roll;
cam_pitch = sensor_parameters.cam_pitch;
% determine which parts are feasible for taking measurements
[F_visible, faces_visible] = get_visible_faces(num_faces, F_points, F_center, ...
    F_normal, cam_pos, cam_roll, cam_pitch, cam_yaw, sensor_parameters);
    

%% Visualization
% surface mesh
fig_main = figure;
hold on;
grid on;
axis([-12 12 -12 12 0 24]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_main = fig_main.CurrentAxes;
daspect(ax_main, [1 1 1]);
view(ax_main, 3);
h_mesh = trimesh(TR);
h_mesh.FaceColor = 'w';
h_mesh.FaceAlpha = 1;
h_mesh.EdgeColor = 'c';
h_mesh.LineWidth = 0.5;
h_mesh.LineStyle = '-';

ax_main.Visible = 'off';

% camera fov
h_cam = plot_camera_fov(ax_main, cam_pos, cam_roll, cam_pitch, cam_yaw, ...
    sensor_parameters.fov_x, sensor_parameters.fov_y, ...
    sensor_parameters.fov_range_max, 'r');

% visible faces
for iFace = 1 : num_faces
    if F_visible(iFace) == 1
        patch(ax_main, 'XData', F_points(iFace, 1, :), ...
              'YData', F_points(iFace, 2, :), ...
              'ZData', F_points(iFace, 3, :), ...
              'FaceColor', 'b', ... 
              'FaceAlpha', 0.5, ...
              'EdgeColor', 'b');
    end
end


%% Ground truth map
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
fig_map = figure;
hold on;
axis([-14 14 -14 14 0 30]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_map = fig_map.CurrentAxes;
daspect(ax_map, [1 1 1]);
view(ax_map, 3);
trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
    TR.Points(:,3), F_value, 'EdgeAlpha', 0);
