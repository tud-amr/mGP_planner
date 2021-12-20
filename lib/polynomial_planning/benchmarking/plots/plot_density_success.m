% This plot attempts to figure out how many segments you need vs.
% density.
figure_path = '/Users/helen/papers/asldoc-2016-iros-oleynikova-replanning/figures/matlab/';
do_print = 0;

% First, select all the relevant trials.
map_densities = [];
total_trials = [];
chomp_successes = [];

for i = 1:length(trials)
  % See where this fits.
  tree_density = trials(i).map.tree_density;
  [ind] = find(map_densities == tree_density, 1);
  if (isempty(ind))
    map_densities(end+1) = tree_density;
    ind = length(map_densities);
    total_trials(ind) = 0;
    chomp_success(ind) = 0;
    pto_success(ind, 1:5) = 0;
    pto_success_restart(ind, 1:5) = 0;
  end
  
  % Now categorize all the trials.
  total_trials(ind) = total_trials(ind) + 1;
  chomp_success(ind) = chomp_success(ind) + trials(i).chomp_trials(1).metrics.success;
  % Iterate.
  for j = 1:length(trials(i).pto_trials)
    random_restarts = trials(i).pto_trials(j).params.random_restarts;
    num_segments = trials(i).pto_trials(j).params.num_segments;
    if (random_restarts)
      pto_success_restart(ind, num_segments) = pto_success_restart(ind, num_segments) + trials(i).pto_trials(j).metrics.success;
    else
      pto_success(ind, num_segments) = pto_success(ind, num_segments) + trials(i).pto_trials(j).metrics.success;
    end
  end
end

figure(3)
clf
% Short-hand test:
mean_trials = mean(total_trials);
figure(3)
plot(map_densities, chomp_success/mean_trials, '--o', ...
   map_densities, pto_success(:, :)/mean_trials, '--x', ...
   map_densities, pto_success_restart(:, :)/mean_trials, '-.s');
legend()

% Long-hand figure setup
figure(4)
plot([0, map_densities], [1, chomp_success/mean_trials], 'k-o', 'LineWidth', 2, 'MarkerSize', 10);
hold on;
plot([0, map_densities], [1; pto_success(:, 1)/mean_trials], 'r--x', 'LineWidth', 2, 'MarkerSize', 10);
plot([0, map_densities], [1; pto_success(:, 5)/mean_trials], 'b--x', 'LineWidth', 2, 'MarkerSize', 10);

plot([0, map_densities], [1; pto_success_restart(:, 1)/mean_trials], 'r-.s', 'LineWidth', 2, 'MarkerSize', 10);
plot([0, map_densities], [1; pto_success_restart(:, 5)/mean_trials], 'b-.s', 'LineWidth', 2, 'MarkerSize', 10);

legend('CHOMP', '1 segment', '5 segments', '1 segments + restarts', '5 segments + restarts', 'Location', 'SouthWest');
xlabel('Tree Density [{trees/25 m^2}]')
ylabel('Success [{fraction}]')
axis([0 20 0 1])
title('Planning Success vs. Tree Density in Poisson Forest');
%set(gca, 'LineWidth', 2); 
set(gca,'FontSize',12);
if (do_print)
  print('-dpng', [figure_path 'success_density.png']);
  print('-depsc', [figure_path 'success_density.eps']);
end
hold off;

%figure(4)
% This next figure is about number of segments vs overall cost. Or maybe
% success. Who knows. Taken over all trials.



%plot(1:5, pto_success(:, :), '--x', 1:5, pto_success_restart(:, :), '-.s');