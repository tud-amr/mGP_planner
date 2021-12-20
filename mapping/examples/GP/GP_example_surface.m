% gp examples on a surface
clear all
clear 
clc 

% rng(1,'twister');


%% Environment
model_name = 'cylinder';
model.name = model_name;
% mesh
data_mesh = load([model_name, '_mesh.mat']);
model.TR = data_mesh.TR;
model.valid_faces = data_mesh.valid_faces;
TR = data_mesh.TR;
% occupancy
data_occupancy = load([model_name, '_map_occupancy']);
model.occupancy = data_occupancy.occupancy; 
% esdf
data_esdf = load([model_name, '_map_esdf']);
model.esdf = data_esdf.esdf; 
% true temperature field
data_temperature_field = load([model_name, '_temperature_field']);
model.temperature_field = data_temperature_field.F_value;


%% Parameters
[map_parameters, sensor_parameters, planning_parameters, optimization_parameters, ...
    matlab_parameters] = load_parameteres(model);


%% Ground truth and initial map
dim_x_env = map_parameters.dim_x_env;
dim_y_env = map_parameters.dim_y_env;
dim_z_env = map_parameters.dim_z_env;
ground_truth_faces_map = create_ground_truth_map(map_parameters);
figure;
subplot(2, 2, 1)
hold on;
axis([-9 9 -9 9 0 25]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Ground truth map')
daspect([1 1 1]);
view(3);
trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
    TR.Points(:,3), ground_truth_faces_map, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet


%% Measurements
% initial_observe_idx = [1:map_parameters.num_faces];
% initial_observe_idx = [257, 273, 289, 305];
% initial_observe_idx = [257, 273, 289, 305, ...
%                        577, 593, 609, 625, ...
%                        897, 913, 929, 945];
initial_observe_idx = [257, 320, 272, 273, 288, 289, 304, 305, ...
                       577, 640, 592, 593, 608, 609, 624, 625, ...
                       897, 960, 912, 913, 928, 929, 944, 945];
num_measure = length(initial_observe_idx);
sigma2_measure = 0.04;
X_ref = zeros(num_measure, 3);
Y_measure = zeros(num_measure, 1);
for i = 1 : num_measure
    X_ref(i, :) = map_parameters.F_center(i, :);
    Y_measure(i) = ground_truth_faces_map(i) + sqrt(sigma2_measure)*randn;
end
faces_map_measure = 0.5*ones(size(ground_truth_faces_map));
faces_map_measure(initial_observe_idx) = ground_truth_faces_map(initial_observe_idx);
subplot(2, 2, 2)
hold on;
axis([-9 9 -9 9 0 25]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Initial observations')
daspect([1 1 1]);
view(3);
trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
    TR.Points(:,3), faces_map_measure, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet
% trimesh(TR, 'FaceColor', 'w', 'FaceAlpha', 0.1, 'EdgeColor', 'c', 'EdgeAlpha', 0.8);
% for i = 1 : num_measure
%     % patch the face
%     F_points_idx_iF = TR.ConnectivityList(initial_observe_idx(i), :);
%     F_points_iF = [ TR.Points(F_points_idx_iF(1), :); ...
%                     TR.Points(F_points_idx_iF(2), :); ...
%                     TR.Points(F_points_idx_iF(3), :)];
%     patch('XData', F_points_iF(:, 1), ...
%           'YData', F_points_iF(:, 2), ...
%           'ZData', F_points_iF(:, 3), ...
%           'FaceColor', [0, 0, Y_measure(i)], ...
%           'FaceAlpha', 1.0, ...
%           'EdgeColor', 'c', ...
%           'EdgeAlpha', 0.8);
% end


%% Training a GP
% specify the mean, covariance and likelihood functions
mean_func = @meanConst;
cov_func = {'covMaterniso', 3}; % covSEiso
lik_func = @likGauss;
inf_func = @infGaussLik;
% initialize the hyperparameter struct
hyp.mean = 0.5;
hyp.cov  = [1.3 0.3];           % hyp = [ log(ell)
                                %         log(sf) ]
hyp.lik  = 0.35;
% optimizing hyperparameter
N = 500;                % maximal number of function evaluations
hyp_opt = minimize(hyp, @gp, -N, inf_func, mean_func, cov_func, lik_func, X_ref, Y_measure);
% hyp_opt = hyp;


%% GP inference
Z = map_parameters.F_center;
% inference
[ymu, ys2, fmu, fs, ~ , post] = gp(hyp_opt, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y_measure, Z);
subplot(2,2,3);
hold on;
axis([-9 9 -9 9 0 25]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title('Mean - prior')
daspect([1 1 1]);
view(3);
trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
    TR.Points(:,3), ymu, 'EdgeAlpha', 0);
caxis([0, 1]);
colormap jet
% colorbar
% subplot(2,3,4);
% hold on;
% axis([-9 9 -9 9 0 25]);
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% title(['Var. - prior. Trace = ', num2str(sum(ys2), 5)])
% daspect([1 1 1]);
% view(3);
% trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
%     TR.Points(:,3), ys2, 'EdgeAlpha', 0);
% var_max = max(ys2);
% caxis([0 var_max]);

%% Calculate covariance
faces_map_prior.m = ymu;
alpha = post.alpha;
L = post.L; 
sW = post.sW; 
Kss = feval(cov_func{:}, hyp_opt.cov, Z);
Ks = feval(cov_func{:}, hyp_opt.cov, X_ref, Z);
% Kss = feval(cov_func, hyp_opt.cov, Z);
% Ks = feval(cov_func, hyp_opt.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
  V = L'\(sW.*Ks);
  faces_map_prior.P = Kss - V'*V;                       % predictive variances
 else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  faces_map_prior.P = Kss + Ks'*LKs;                    % predictive variances
end
subplot(2,2,4);
hold on;
axis([-9 9 -9 9 0 25]);
xlabel('x [m]');
ylabel('y [m]');
zlabel('z [m]');
title(['Var. - prior. Trace = ', num2str(trace(faces_map_prior.P), 5)])
daspect([1 1 1]);
view(3);
trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
    TR.Points(:,3), diag(faces_map_prior.P), 'EdgeAlpha', 0);
var_max = max(diag(faces_map_prior.P));
caxis([0 var_max]);

% subplot(2,3,6);
% hold on;
% axis([-9 9 -9 9 0 25]);
% xlabel('x [m]');
% ylabel('y [m]');
% zlabel('z [m]');
% title(['Var. - prior. Trace = ', num2str(trace(Kss), 5)])
% daspect([1 1 1]);
% view(3);
% trisurf(TR.ConnectivityList, TR.Points(:,1), TR.Points(:,2), ...
%     TR.Points(:,3), diag(Kss), 'EdgeAlpha', 0);
% caxis([0 var_max]);
% colorbar
