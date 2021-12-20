% compute geodesic distance between vertices of the mesh surface
close all
clear all
clear 
clc 

root_folder = pwd;

%% load file
model_name = 'cylinder';            % cylinder, boeing747, u_cylinder
data_mesh = load([model_name, '_mesh.mat']);  
TR = data_mesh.TR;


%% initialize computation
global geodesic_library;                
geodesic_library = 'geodesic_release';                %"release" is faster and "debug" does additional checks
vertices = TR.Points;
faces = TR.ConnectivityList;
num_vertices = size(TR.Points, 1);
num_faces = size(TR.ConnectivityList, 1);
mesh = geodesic_new_mesh(vertices,faces);           %initilize new mesh
algorithm = geodesic_new_algorithm(mesh, 'exact'); 	%initialize new geodesic algorithm


%% computation for each loop
% create an io file
% geo_dis_mtx_obj = matfile([root_folder, '/surface_resources/cylinder/model/', ...
%     model_name, '_vertice_geo_distance.mat'], 'Writable', true);
% geodesic distance between each pair of vertices
vertice_geo_dis_mtx = zeros(num_vertices, num_vertices);
for i = 1 : 1 : num_vertices
    for j = i+1 : 1 : num_vertices
        try
            vertex_i = i;
            vertex_j = j;         % both choose the first vertex of the face
            source_points = {geodesic_create_surface_point('vertex',vertex_i,vertices(vertex_i,:))};
            geodesic_propagate(algorithm, source_points);
            destination = geodesic_create_surface_point('vertex',vertex_j,vertices(vertex_j,:));
            path = geodesic_trace_back(algorithm, destination);	
            [x,y,z] = extract_coordinates_from_path(path);
            path_length = sum(sqrt(diff(x).^2 + diff(y).^2 + diff(z).^2));	% length of the path
            vertice_geo_dis_mtx(i, j) = path_length;
            vertice_geo_dis_mtx(j, i) = path_length;
        catch
            warning('Strang problem encountered!');
            vertice_geo_dis_mtx(i, j) = 1E4;
            vertice_geo_dis_mtx(j, i) = 1E4;
        end
    end
    fprintf('i = %d, %.2f %% finished. Please wait... \n', ...
        i, 100*i/num_vertices);
end

% further compute geodesic distance between each pair of faces

save([root_folder, '/surface_resources/',model_name,'/model/', ...
    model_name, '_vertice_geo_distance.mat'], 'vertice_geo_dis_mtx');
