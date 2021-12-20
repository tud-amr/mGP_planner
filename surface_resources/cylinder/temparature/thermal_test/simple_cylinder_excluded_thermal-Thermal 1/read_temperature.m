clear all
clear 
clc 

%% read simulated temperature field
filename = 'simple_cylinder_excluded_thermal-Thermal 1-Results-Thermal1-2.csv';
data_table = readtable(filename);
data_array = table2array(data_table);
num_data = size(data_array, 1);

%% temprature data, [n x 4]
% original data
temperature_array = data_array(:, 2:5);     % temp, x, y, z
temperature_array(:, 2:4) = temperature_array(:, 2:4)/100;   % convert to m
% coordinate translation
obj_center = [6; 6; 0];
temperature_array(:, 2:4) = temperature_array(:, 2:4) + obj_center';
x_interval = [min(temperature_array(:,2)), max(temperature_array(:,2))];
y_interval = [min(temperature_array(:,3)), max(temperature_array(:,3))];
z_interval = [min(temperature_array(:,4)), max(temperature_array(:,4))];
t_interval = [min(temperature_array(:,1)), max(temperature_array(:,1))];
dim_x_env = [-8, 20];
dim_y_env = [-8, 20];
dim_z_env = [0, 30];

%% visulization
% geometry
figure;
grid on;
hold on;
axis([dim_x_env, dim_y_env, dim_z_env]);
view(3);
daspect([1 1 1])
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
scatter3(temperature_array(:, 2), temperature_array(:, 3), temperature_array(:, 4), ...
    254, 'b', 'MarkerFaceColor', 'b', 'MarkerFaceAlpha', 1.0, ...
    'MarkerEdgeColor', 'b', 'MarkerEdgeAlpha', 0.0);
% temparature distribution
figure;
grid on;
hold on;
axis([dim_x_env, dim_y_env, dim_z_env]);
view(3);
daspect([1 1 1])
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
scatter3(temperature_array(:, 2), temperature_array(:, 3), temperature_array(:, 4), ...
    128, temperature_array(:, 1), 'filled');
colormap jet
colorbar;

%% surface interpolation
X = temperature_array(:, 2);
Y = temperature_array(:, 3);
Z = temperature_array(:, 4);
% generate the surface
DT = delaunayTriangulation(X, Y, Z);
tri = DT.ConnectivityList;
xi = DT.Points(:,1); 
yi = DT.Points(:,2);
zi = DT.Points(:,3);
% get point temperature
% num_points = size(DT.Points, 1);
% temperature_points = zeros(num_points, 1);
% for i = 1 : num_points
%     pos_val = DT.Points(i, :);
%     % find the cloest from raw data
%     [pos_closest, idx_closest] = find_closest_point(pos_val, temperature_array(:, 2:4));
%     temperature_points(i) = temperature_array(idx_closest, 1);
% end
% surfplot
figure;
grid on;
hold on;
axis([dim_x_env, dim_y_env, dim_z_env]);
view(3);
daspect([1 1 1])
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
trisurf(tri, xi, yi, zi, 'FaceColor', 'w',  ...
    'EdgeColor', 'c', 'EdgeAlpha', 1.0);


