% This plot attempts to figure out how many segments you need vs.
% density.
% But it sucks so don't use it.
figure_path = '/Users/helen/papers/asldoc-2016-iros-oleynikova-replanning/figures/matlab/';
do_print = 0;

% First, generate a map.
%seed = seed+1;
seed = 58
rng(seed);
map = create_forest_map(1, 15, 5, 1);

% Select start and end points (always the same per trial).
start_point = [0.5 0.5];
goal_point = [4.5, 4.5];
visualize = 0;



%%
figure(1)
clf;
axis square;
cost_map = get_cost_map(map);
map_table = get_map_table(map);
plot_map_table(map, cost_map);
cmap = flipud(colormap('gray'));
cmap(3*end/10:3*end/10+1, 3) = 1.0;
cmap(3*end/10:3*end/10+1, 2) = 0.5;
cmap(3*end/10:3*end/10+1, 1) = 0.0;

%cmap(1:1*end/4, 1) = 0.5;
colormap(cmap)
c = colorbar;
caxis([0 1])
%%
hold all;
%%

[chomp_path, chomp_metrics] = evaluate_toolbox_chomp(map, start_point, goal_point, visualize);
plot(chomp_path(:, 1), chomp_path(:, 2), 'LineWidth', 2);
[traj_out, cont_metrics] = evaluate_continuous(map, start_point, ...
          goal_point, 0, 1, 0);
[t, p] = sample_trajectory(traj_out, 0.1);
plot(p(:, 1), p(:, 2), 'LineWidth', 2);

[traj_out, cont_metrics] = evaluate_continuous(map, start_point, ...
          goal_point, visualize, 5, 0);
[t, p] = sample_trajectory(traj_out, 0.1);
plot(p(:, 1), p(:, 2), 'LineWidth', 2);

title('Typical Solution Paths')
xlabel('x position [meters]')
ylabel('y position [meters]')
ylabel(c, 'Collision Cost')
legend('CHOMP', 'Our method - 1 segment', 'Our method - 5 segments', 'Location', 'SouthEast')
        
%plot(map_densities, chomp_success/mean_trials, '--o', ...
%   map_densities, pto_success(:, [1,5])/mean_trials, '--x', ...
%   map_densities, pto_success_restart(:, [1,5])/mean_trials, '-.s');
% 
% % Long-hand figure setup
% plot([0, map_densities], [1, chomp_success/mean_trials], 'k-o', 'LineWidth', 2, 'MarkerSize', 10);
% hold on;
% plot([0, map_densities], [1; pto_success(:, 1)/mean_trials], 'r--x', 'LineWidth', 2, 'MarkerSize', 10);
% plot([0, map_densities], [1; pto_success(:, 5)/mean_trials], 'b--x', 'LineWidth', 2, 'MarkerSize', 10);
% 
% plot([0, map_densities], [1; pto_success_restart(:, 1)/mean_trials], 'r-.s', 'LineWidth', 2, 'MarkerSize', 10);
% plot([0, map_densities], [1; pto_success_restart(:, 5)/mean_trials], 'b-.s', 'LineWidth', 2, 'MarkerSize', 10);
% 
% legend('CHOMP', '1 segment', '5 segments', '1 segments + restarts', '5 segments + restarts', 'Location', 'SouthWest');
% xlabel('Tree Density [{trees/10 m^2}]')
% ylabel('Success [{fraction}]')
% axis([0 20 0 1])
% title('Planning Success vs. Tree Density in Poisson Forest');
% %set(gca, 'LineWidth', 2); 
set(gca,'FontSize',12);
if (do_print)
   print('-dpng', [figure_path 'typical_paths.png']);
   print('-depsc', [figure_path 'typical_paths.eps']);
end
hold off;


%plot(1:5, pto_success(:, :), '--x', 1:5, pto_success_restart(:, :), '-.s');