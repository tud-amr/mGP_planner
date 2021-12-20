function [trajectory_out, iterations, final_cost, initial_cost] = optimize_trajectory_collisions(map, trajectory, visualize, random_perturb, random_perturb_mag)

if (nargin < 3)
  visualize = 1;
end
if (nargin < 4)
  random_perturb = 0;
end
if (nargin < 5)
  random_perturb_mag = 0.1;
end
%OPTIMIZE_PATH_COLLISIONS Summary of this function goes here
%   Detailed explanation goes here

%% Figure out if we should do anything
%[colliding_times, colliding_segments] = is_trajectory_in_collision(map, trajectory);
%if (isempty(colliding_times))
%  trajectory_out = trajectory;
%  return
%end

%% Generate cost maps and matrices.
[cost_map, cost_map_x, cost_map_y] = get_cost_map(map);

if (visualize)
  plot_map_table(map, get_map_table(map));
end

% What is the cost of the current trajectory?
% 2 parts - J_smooth and J_collision
% J_der is axis-independent, J_collision is axis-dependent.

% Should cache the A_inv and M matrices for each axis, as they don't
% change.
Ms = {};
D_Fs = {};
A_invs = {};
R_unordereds = {};
x0 = [];
for k = 1:trajectory.K
  [Ms{k}, D_Fs{k}] = assemble_M_and_D_F(trajectory.vertices_structs(k));
  [A_invs{k}, R_unordereds{k}] = assemble_A_and_R(trajectory.vertices_structs(k).times, trajectory.N);

  total_ds = size(Ms{k}, 2);

  n = trajectory.N + 1;
    
  p = [];
  for i = 1:length(trajectory.segments)
    p((i-1)*n+1:i*n, 1) = trajectory.segments(i).coefficients(:, k);
  end
  total_num_fixed = trajectory.vertices_structs(k).num_fixed;
  M = Ms{k};
  A_inv = A_invs{k};
  D_P = pinv(M)*inv(A_inv)*p;
  D_P = D_P(total_num_fixed+1:total_ds);
  x0(end+1:end+length(D_P)) = D_P;
end

if (random_perturb)
  x0 = x0 + random_perturb_mag*(rand(size(x0))-0.5);
end

%% Get the costs of the initial trajectory.
[initial_cost, initial_grad] = get_trajectory_cost(x0, trajectory, map,  cost_map, cost_map_x, cost_map_y, Ms, D_Fs, A_invs, R_unordereds);

%%
cost_func = @(x0)get_trajectory_cost(x0, trajectory, map,  cost_map, cost_map_x, cost_map_y, Ms, D_Fs, A_invs, R_unordereds);
tic

%options = optimoptions('fminunc','Algorithm','trust-region','GradObj','on','Display','iter','DerivativeCheck','off', 'TolFun', 1e-4);
options = optimoptions('fminunc','Algorithm','quasi-newton','GradObj','on','Display','iter','DerivativeCheck','off', 'TolFun', 1e-4, 'MaxIter', 25);
%options = optimoptions('fminunc','Algorithm','quasi-newton','GradObj','off','Display','iter','DerivativeCheck','off', 'TolFun', 1e-4, 'MaxIter', 15);

[x, final_cost, ~, output] = fminunc(cost_func, x0, options);
iterations = output.iterations;

% Quickly implement gradient descent...
% x = x0;
% lambda = 100;
% for i = 1:50
%    [cost, grad] = cost_func(x);
%    x = x - 1/lambda * grad;
%    disp(sprintf('Iter: %d cost: %f grad norm: %f', i, cost, norm(grad)));
% end
% final_cost = cost;
% toc
% iterations = 0;



% tic
% options2 = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt','Display','iter','Jacobian','on');
% [x2] = lsqnonlin(cost_func, x0, [], [], options2);
% final_cost2 = cost_func(x2);
% final_cost2
% toc

disp(sprintf('Initial cost: %f, final cost: %f\n', initial_cost, final_cost));

%x = x2;
%% Try by hand
% x3 = x0;
% for i = 1:100
%   [cost, grad] = cost_func(x3);
%   cost
%   x3 = x3 - 1/10*grad;
% end  
% x = x3;
%% Test jacobians
% [jac, err] = jacobianest(cost_func, x0);
% [cost, jac2] = cost_func(x0);
% 
% %gradient_error = norm(jac2 - jac)
% gradient_ratio = jac2./jac

%% Resolve the x vector as coefficients on the trajectory.
current_index = 1;
trajectory_out = trajectory;
for k = 1:trajectory.K
  n = trajectory.N + 1;
  p = [];
  
  num_fixed = trajectory.vertices_structs(k).num_fixed;
  num_free = size(Ms{k}, 2) - num_fixed;
  D_P = x(current_index : current_index + num_free - 1)';
  current_index = current_index + num_free;
  
  coeffs = A_invs{k}*Ms{k}*[D_Fs{k};D_P];
  
  for i = 1 : trajectory.num_elements-1
    start_index = (i-1) * (trajectory.N + 1) + 1;
    end_index = i * (trajectory.N + 1);
    trajectory_out.segments(i).coefficients(:, k) = coeffs(start_index:end_index);
    %trajectory.segments(i).time = trajectory.vertices_structs(j).times(i+1);
  end
end

%% Plot that trajectory
if (visualize)
  plot_trajectory(trajectory_out)
end
end