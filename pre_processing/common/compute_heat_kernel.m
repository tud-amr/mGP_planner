% a heat kernel approximation example
clear all 
close all
clear 
clc 

root_folder = pwd;

%% Load mesh file
model_name = 'cylinder';       % cylinder, boeing747, ucylinder
data_mesh = load([model_name, '_mesh.mat']);
model.TR = data_mesh.TR;
TR = data_mesh.TR;
figure;
hold on;
trimesh(TR, 'FaceColor', [0.8,0.8,0.8], 'FaceAlpha', 1.0, 'EdgeColor', 'k', 'EdgeAlpha', 0.2, ...
    'LineWidth', 1.0, 'LineStyle', '-');
daspect([1 1 1]);
view(3);


%% Mesh pre-processing
mesh = mesh_preporcessing(TR);


%% Monte Carlo sampling
test_numF = mesh.numF;
dt = 1;
speed = 0.25;
T = 100;
N = 100000;
heat_kernel = zeros(test_numF, test_numF, T);
% loop for each faceId
for iF = 1 : test_numF
    s0_faceId = iF;
    path_all = zeros(3, ceil(T/dt)+1, N);          % record paths of all samples
    faceID_path_all = zeros(1, ceil(T/dt)+1, N);
    % initialize particles
    particle.local_r = [1/3; 1/3];                      % 2x1
    particle.meshFaceIdx = s0_faceId;
    particle.r = mesh.coord_l2g(particle.meshFaceIdx).Jacobian*particle.local_r + ...
        mesh.coord_l2g(particle.meshFaceIdx).base;      % 3x1
    particle.vel = zeros(3, 1);
    % sampling
    particle_all = repmat(particle, [N, 1]);
    particle_end_all = particle_all;
    parfor iS = 1 : N
        [particle_i, path_i, faceID_path_i] = particle_move_multiple_steps(particle_all(iS), mesh, dt, speed, T);
        particle_end_all(iS) = particle_i;
        path_all(:, :, iS) = path_i;
        faceID_path_all(:, :, iS) = faceID_path_i;
    end
    % construct current kernel
    heat_kernel_s0 = zeros(test_numF, T);
    for t = 1 : T
        faceId_t_all = reshape(faceID_path_all(:, t, :), [1, N]);   % 1xN
        for j = 1 : test_numF
            % by accumulating those pos_t in the i face
            heat_kernel_s0(j, t) = (1/mesh.F_area(1, j)) ...
                * length(find(faceId_t_all == j))/N;
        end
    end
    % merge to current one
    heat_kernel(iF, :, :) = heat_kernel_s0;
end

save([root_folder, '/surface_resources/',model_name,'/model/', ...
    model_name, '_heat_kernel.mat'], 'heat_kernel');
