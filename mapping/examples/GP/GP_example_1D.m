% one-dimentional gp examples
clear all
clear 
clc 

rng(1,'twister');

%% Ground truth function
a = 0.3; 
b = 1.2;
f = @(x) a*x + b + sin(x);
x_g = 0 : 0.1 : 5;
y_g = f(x_g);
figure;
hold on; box on; grid on;
axis([0 5 0 4]);
plot(x_g, y_g, '-b', 'LineWidth', 1.5);
% title('Ground truth fuction');


%% Measurements
n_t = 20;               % number of training points
s2_t = 0.1;              % observation noise
x_t = 5*rand(n_t, 1);   % training points inputs, x
y_t = f(x_t) + sqrt(s2_t)*randn(n_t, 1);    % noisy training points targets, y
% plot training samples
scatter(x_t, y_t, 64, 'r', '+');


%% Training a GP
% specify the mean, covariance and likelihood functions
mean_func = [];
cov_func = @covSEiso;
lik_func = @likGauss;
inf_func = @infGaussLik;
% initialize the hyperparameter struct
hyp.mean = [];
hyp.cov  = [0 0];
hyp.lik  = 0;
% optimizing hyperparameter
N = 100;                % maximal number of function evaluations
hyp_opt = minimize(hyp, @gp, -N, inf_func, mean_func, cov_func, lik_func, x_t, y_t);
% hyp_opt = hyp;


%% GP inference
x_s = x_g';
[ymu, ys2] = gp(hyp_opt, inf_func, mean_func, cov_func, lik_func, x_t, y_t, x_s);
plot(x_s, ymu, '-r', 'LineWidth', 1.5, 'LineStyle', '--');
yf = [ymu + 2*sqrt(ys2); flip(ymu - 2*sqrt(ys2),1)];
fill([x_s; flip(x_s,1)], yf, [7 7 7]/8, 'FaceAlpha', 0.5, 'EdgeAlpha', 0.8);
legend('Ground truth', 'Training samples', 'Prediction', 'Std')

