close all
clear all
clear 
clc

global geodesic_library;                
geodesic_library = 'geodesic_release';                %"release" is faster and "debug" does additional checks

%% Load a mesh
data = load('cylinder_mesh.mat');
TR = data.TR;

%% Compute the shortest path
vertices = TR.Points;
faces = TR.ConnectivityList;
mesh = geodesic_new_mesh(vertices,faces);           %initilize new mesh
algorithm = geodesic_new_algorithm(mesh, 'exact'); 	%initialize new geodesic algorithm
vertex_i = 10;                                      %create a single source at vertex #1
source_points = {geodesic_create_surface_point('vertex',vertex_i,vertices(vertex_i,:))};
geodesic_propagate(algorithm, source_points);       %propagation stage of the algorithm (the most time-consuming)
vertex_j = 820;                                    %create a single destination at vertex #N
destination = geodesic_create_surface_point('vertex',vertex_j,vertices(vertex_j,:));
path = geodesic_trace_back(algorithm, destination);	%find a shortest path from source to destination
[x,y,z] = extract_coordinates_from_path(path);
path_length = sum(sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2));            %length of the path
path_length

%% Visualization
hold off;
colormap('default');
trimesh(TR, 'FaceColor', [0.8,0.8,0.8], 'FaceAlpha', 1.0, ...
    'EdgeColor', 'k', 'EdgeAlpha', 0.2, ...
    'LineWidth', 1.0, 'LineStyle', '-');
daspect([1 1 1]);

hold on;
plot3(source_points{1}.x, source_points{1}.y, source_points{1}.z, 'or', 'MarkerSize',3);    %plot sources

plot3(destination.x, destination.y, destination.z, 'ok', 'MarkerSize',3);       %plot destination 
h = plot3(x*1.001,y*1.001,z*1.001,'k-','LineWidth',2);    %plot path
legend(h,'geodesic curve');

geodesic_delete;

