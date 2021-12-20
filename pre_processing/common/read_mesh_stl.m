% read mesh info from .stl file and save to .mat
clear all
clear 
clc 

root_folder = pwd;

%% read file
model_name = 'cylinder';            % cylinder, boeing747, u_cylinder
switch model_name
    case 'cylinder'
        file_name = 'cylinder_sw_gmsh.stl'; % simple_cylinder_solid, cylinder_sw_gmsh
    otherwise
        error('Model cannot be found!');
end
[TR, fileformat, attributes, solidID] = stlread(file_name);


%% Valid faces for inspection (ignore the bottom side)
valid_faces = [];
bottom_side = [];
for i = 1 : size(TR.ConnectivityList, 1)
    vertices_i = TR.Points(TR.ConnectivityList(i, :), :);
    if max(vertices_i(:, 3)) <= 0.2
        bottom_side = [bottom_side; i];
    else
        valid_faces = [valid_faces; i];
    end
end
% for debugging
% valid_faces = 1 : size(TR.ConnectivityList, 1);
% bottom_side = [];


%% Save data
save([root_folder, '/surface_resources/',model_name,'/model/', ...
    model_name, '_mesh.mat'], 'TR', 'valid_faces', 'bottom_side');


%% visualization
% figure
data_mesh = load([model_name, '_mesh.mat']);
model.TR = data_mesh.TR;
TR = data_mesh.TR;
fig_mesh = figure;
hold on;
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
ax_mesh = fig_mesh.CurrentAxes;
ax_mesh.Visible = 'off';
% trimesh(TR, 'FaceColor', [0.8,0.8,0.8], 'FaceAlpha', 1.0, ...
%     'EdgeColor', 'k', 'EdgeAlpha', 0.2, ...
%     'LineWidth', 1.0, 'LineStyle', '-');
trimesh(TR, 'FaceColor', 'w', 'FaceAlpha', 1.0, ...
    'EdgeColor', 'c', 'EdgeAlpha', 0.8, ...
    'LineWidth', 0.5, 'LineStyle', '-');
daspect([1 1 1]);
view(3);
% test
num_faces = size(TR.ConnectivityList, 1);
num_vertices = size(TR.Points, 1);
F_normal = faceNormal(TR);
F_center = incenter(TR);
for iF = 1 : num_faces
%     pause(0.05);
%     pause;
%     disp(['Face ', num2str(iF)]);
    % face triangle points
    F_points_idx_iF = TR.ConnectivityList(iF, :);
    F_points_iF = [ TR.Points(F_points_idx_iF(1), :); ...
                    TR.Points(F_points_idx_iF(2), :); ...
                    TR.Points(F_points_idx_iF(3), :)];
    F_center_iF = F_center(iF, :);
    F_normal_iF = F_normal(iF, :);
    % patch the face
%     patch(ax_mesh, 'XData', F_points_iF(:, 1), ...
%           'YData', F_points_iF(:, 2), ...
%           'ZData', F_points_iF(:, 3), ...
%           'FaceColor', [0,102,162]/256, ...
%           'FaceAlpha', 1.0, ...
%           'EdgeColor', 'c');
    % plot face center
%     plot3(ax_mesh, F_center_iF(1), F_center_iF(2), ...
%         F_center_iF(3), '.r');
    % plot the normal
%     quiver3(ax_mesh, F_center_iF(:,1), F_center_iF(:,2), F_center_iF(:,3), ...
%         F_normal_iF(:,1), F_normal_iF(:,2), F_normal_iF(:,3), ...
%         0.5, 'color', 'r');
end
