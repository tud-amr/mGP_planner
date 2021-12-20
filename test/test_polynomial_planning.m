clear all
clear 
clc 

%% Parameters
dim_x_env = [-14, 14];
dim_y_env = [-14, 14];
dim_z_env = [0, 30];
max_vel = 4;
max_acc = 3;

%% Environment
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
% visualize
fig_map = figure;
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
ax_map = fig_map.CurrentAxes;
% mesh object
h_mesh = trimesh(TR);
h_mesh.FaceColor = 'w';
h_mesh.FaceAlpha = 1;
h_mesh.EdgeColor = 'c';
h_mesh.LineWidth = 0.5;
h_mesh.LineStyle = '-';
axis([dim_x_env dim_y_env dim_z_env]);
daspect(ax_map, [1 1 1]);
view(ax_map, 3);

%% 3D trajectory in obstacle-free env.
% waypoints, the first one is the starting point
waypoints = [-6,     -6,    4,      deg2rad(45);
             -4,     -6,    24.54,  deg2rad(60);
              4,     -1,    23,     deg2rad(170);
             10,   -2.4,    16,    deg2rad(170)];
% plan trajectory and sample, position
trajectory = plan_path_waypoints(waypoints(:,1:3), max_vel, max_acc);
dt = 0.5;
[t, p] = sample_trajectory(trajectory, dt);
% yaw trajectory
segment_time = zeros(trajectory.num_elements, 1);
for i = 2 : trajectory.num_elements
    segment_time(i) = trajectory.segments(i-1).time;
end
yaw_trajectory = plan_yaw_waypoints(waypoints(:,4), segment_time);
[t_yaw, yaw] = sample_trajectory(yaw_trajectory, dt);
% visualize trajectory
% figure;
hold on;
grid on;
axis([dim_x_env dim_y_env dim_z_env]);
scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 200, 'xk');
for i = 1 : size(waypoints, 1)
    u = 4*cos(waypoints(i,4));
    v = 4*sin(waypoints(i,4));
    w = 0;
    quiver3(waypoints(i,1), waypoints(i,2), waypoints(i,3), u, v, w, ...
        'Color', 'r', 'LineWidth', 2.0, 'MaxHeadSize', 0.8);
end
h = plot_trajectory_cline(t, p, yaw);
daspect([1 1 1]);
view(3);


