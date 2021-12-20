% two-dimentional gp examples
clear all
clear 
clc 

% rng(1,'twister');

%% Ground truth map
% Environment
cluster_radius = 3;
% Dimensions [m]
dim_x_env = 10;
dim_y_env = 10;
% Map resolution [m/cell]
map_parameters.resolution = 0.5;
% Map dimensions [cells]
map_parameters.dim_x = dim_x_env/map_parameters.resolution;
map_parameters.dim_y = dim_y_env/map_parameters.resolution;
dim_x = map_parameters.dim_x;
dim_y = map_parameters.dim_y;
[mesh_x,mesh_y] = meshgrid(linspace(1,dim_x,dim_x), linspace(1,dim_y,dim_y));
% Prediction map dimensions [cells]
predict_dim_x = dim_x*1;
predict_dim_y = dim_y*1;
% Generate (continuous) ground truth map.
ground_truth_map = create_continuous_map(dim_x, dim_y, cluster_radius);
% Visualization
figure;
subplot(4,2,1);
imagesc(ground_truth_map);
caxis([0, 1]);
title('Ground truth map');
set(gca,'Ydir', 'Normal');


%% Measurements
% observations
training_ind_x = [1:20];
training_ind_y = [1:20];
% training_ind_x = [5, 10, 15];
% training_ind_y = [5, 10, 15];
% training_ind_x = [3, 8, 13, 18];
% training_ind_y = [3, 8, 13, 18];
% training_ind_x = [2, 6, 10, 14, 18];
% training_ind_y = [2, 6, 10, 14, 18];
% training_ind_x = [1, 4, 7, 10, 13, 16, 19];
% training_ind_y = [1, 4, 7, 10, 13, 16, 19];
X_ref = [reshape(mesh_x(training_ind_x,training_ind_x), ...
    numel(mesh_x(training_ind_x,training_ind_x)), 1), ...
    reshape(mesh_y(training_ind_y,training_ind_y), ...
    numel(mesh_y(training_ind_y,training_ind_y)), 1)];
num_measure = size(X_ref, 1);
sigma2_measure = 0.1^2;
Y = reshape(ground_truth_map(training_ind_y,training_ind_x),[],1);
Y = Y + sqrt(sigma2_measure)*randn(num_measure, 1);
% visualization
grid_map_measure = 0.5*ones(size(ground_truth_map));
Y_measure = [];
for i = 1 : length(training_ind_x)
    for j = 1 : length(training_ind_y)
        y_measure = ground_truth_map(training_ind_y(j),training_ind_x(i));
        y_measure = y_measure + sqrt(sigma2_measure)*randn;
        grid_map_measure(training_ind_y(j),training_ind_x(i)) = y_measure;
        Y_measure = [Y_measure; y_measure];
    end
end
% Y and Y_measure should be the same with noise
subplot(4,2,2);
imagesc(grid_map_measure);
caxis([0, 1]);
title('Initial observations');
set(gca,'Ydir', 'Normal');


%% Training a GP
% specify the mean, covariance and likelihood functions
mean_func = @meanConst;
cov_func = @covSEiso; %{'covMaterniso', 3}; % covSEiso
lik_func = @likGauss;
inf_func = @infGaussLik;
% initialize the hyperparameter struct
hyp.mean = 0.5;
hyp.cov  = [0 0];
hyp.lik  = 0;
% optimizing hyperparameter
N = 500;                % maximal number of function evaluations
hyp_opt = minimize(hyp, @gp, -N, inf_func, mean_func, cov_func, lik_func, X_ref, Y_measure);
% hyp_opt = hyp;


%% GP inference
[mesh_x,mesh_y] = meshgrid(linspace(1,predict_dim_x,predict_dim_x), ...
    linspace(1,predict_dim_y,predict_dim_y));
Z =  [reshape(mesh_x, numel(mesh_x), 1), reshape(mesh_y, numel(mesh_y), 1)];
% inference
[ymu, ys2, fmu, fs, ~ , post] = gp(hyp_opt, inf_func, mean_func, cov_func, lik_func, ...
    X_ref, Y_measure, Z);
ymu = reshape(ymu, predict_dim_y, predict_dim_x);
ys2 = reshape(ys2, predict_dim_y, predict_dim_x);
subplot(4,2,3);
imagesc(ymu);
caxis([0, 1]);
title('Mean - prior');
set(gca,'Ydir', 'Normal');
subplot(4,2,4);
imagesc(ys2);
title(['Var. - prior. Trace = ', num2str(sum(ys2, 'all'), 5)])
colorbar;


%% Calculate covariance, weird!
grid_map_prior.m = ymu;             
alpha = post.alpha;
L = post.L; 
sW = post.sW; 
% Kss = feval(cov_func{:}, hyp_opt.cov, Z);
% Ks = feval(cov_func{:}, hyp_opt.cov, X_ref, Z);
Kss = feval(cov_func, hyp_opt.cov, Z);
Ks = feval(cov_func, hyp_opt.cov, X_ref, Z);
Lchol = isnumeric(L) && all(all(tril(L,-1)==0)&diag(L)'>0&isreal(diag(L))');
if Lchol    % L contains chol decomp => use Cholesky parameters (alpha,sW,L)
  V = L'\(sW.*Ks);
  grid_map_prior.P = Kss - V'*V;                       % predictive variances
 else                % L is not triangular => use alternative parametrisation
  if isnumeric(L), LKs = L*(Ks); else LKs = L(Ks); end    % matrix or callback
  grid_map_prior.P = Kss + Ks'*LKs;                    % predictive variances
end
subplot(4,2,5);
imagesc(grid_map_prior.P);
title('Covariance - prior');
colorbar;
subplot(4,2,6);
imagesc(reshape(diag(grid_map_prior.P), predict_dim_y, predict_dim_x));
title(['Var. - prior. Trace = ', num2str(trace(grid_map_prior.P), 5)])
colorbar;


%% Another way
subplot(4,2,7);
imagesc(Kss);
title('Covariance - prior');
colorbar;
subplot(4,2,8);
imagesc(reshape(diag(Kss), predict_dim_y, predict_dim_x));
title(['Var. - prior. Trace = ', num2str(trace(Kss), 5)])
colorbar;


