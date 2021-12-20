% compute geodesic distance between triangle centers of the surface
close all
clear all
clear 
clc 

root_folder = pwd;

%% load file
model_name = 'cylinder';            % cylinder, boeing747, u_cylinder
data_mesh = load([model_name, '_mesh.mat']);  
TR = data_mesh.TR;
data_vertice_geo_dis_mtx = load([model_name, '_vertice_geo_distance.mat']);
vertice_geo_dis_mtx = data_vertice_geo_dis_mtx.vertice_geo_dis_mtx;


%% compute distance between faces
num_faces = size(TR.ConnectivityList, 1);
num_vertices = size(TR.Points, 1);
vertices = TR.Points;
F_normal = faceNormal(TR);
F_center = incenter(TR);
face_geo_dis_mtx = zeros(num_faces, num_faces);
for i = 1 : num_faces
    for j = i+1 : num_faces
        % vertices num of i and j
        verts_i = TR.ConnectivityList(i, :);
        verts_j = TR.ConnectivityList(j, :);
        % find the pair with closest geo distance
        dis_ij_min = 1E4;
        geo_idx_i = 0;
        geo_idx_j = 0;
        for idx_i = 1 : 3
            for idx_j = 1 : 3
                dis_ij = vertice_geo_dis_mtx(verts_i(idx_i), verts_j(idx_j));
                if dis_ij <= dis_ij_min
                    dis_ij_min = dis_ij;
                    geo_idx_i = verts_i(idx_i);
                    geo_idx_j = verts_j(idx_j);
                end
            end
        end
        % compute the distance between face center and vertice
        face_geo_dis_mtx(i, j) = dis_ij_min + ...
            norm(F_center(i,:) - vertices(geo_idx_i,:)) + ...
            norm(F_center(j,:) - vertices(geo_idx_j,:));
        face_geo_dis_mtx(j, i) = face_geo_dis_mtx(i, j);
    end
end

save([root_folder, '/surface_resources/',model_name,'/model/', ...
    model_name, '_face_geo_distance.mat'], 'face_geo_dis_mtx');
