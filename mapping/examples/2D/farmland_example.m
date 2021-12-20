clear all;
close all;
clc;

%% Environment
% cluster radius
cluster_radius = 3;
% dimensions [m]
dim_x_env = 30;
dim_y_env = 30;

%% Map settings
% map resolution [m/cell]
map_parameters.resolution = 0.75;
% Map dimensions [cells]
map_parameters.dim_x = dim_x_env/map_parameters.resolution;
map_parameters.dim_y = dim_y_env/map_parameters.resolution;
% position of map in the environment [m]
map_parameters.position_x = -dim_x_env / 2;
map_parameters.position_y = -dim_y_env / 2;
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
% prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;

%% Ground truth map
ground_truth_map = create_continuous_map(dim_x, dim_y, cluster_radius);

