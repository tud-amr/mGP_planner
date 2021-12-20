close all
clear all
clear 
clc

%% Create Delaunay Triangulation example
% a sphere example
radius = 12;
resolution_theta = deg2rad(30);
resolution_phi = deg2rad(30);
num_theta = 2*pi / resolution_theta;
num_phi = 2*pi / resolution_phi;
xyz_P = [];
figure;
hold all;
box on;
axis equal;
for i_theta = 1 : num_theta
    theta_P_temp = resolution_theta*(i_theta-1);
    for j_phi = 1 : num_phi
        phi_P_temp = resolution_phi*(j_phi-1);
        temp_P = [radius*cos(theta_P_temp)*cos(phi_P_temp), ...
                  radius*cos(theta_P_temp)*sin(phi_P_temp), ...
                  radius*sin(theta_P_temp)];
        xyz_P = [xyz_P; temp_P];
        plot3(temp_P(1), temp_P(2), temp_P(3), '.r')
    end
end
DT_sphere = delaunayTriangulation(xyz_P);
tetramesh(DT_sphere,'FaceAlpha',0.3);


