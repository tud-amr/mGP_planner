close all
clear all
clear 
clc 

%% Environment
cluster_radius = 2;
% Dimensions [m]
dim_x_env = 40;
dim_y_env = 40;
dim_z_env = 40;
resolution = 0.5;
dim_x = dim_x_env / resolution;
dim_y = dim_y_env / resolution;
dim_z = dim_z_env / resolution;

ground_truth_map = create_continuous_map_3D(dim_x, dim_y, dim_z, cluster_radius);

%% Visualization
map_1 = ground_truth_map(:,:,1);
imagesc(map_1);
set(gca,'Ydir','Normal');



