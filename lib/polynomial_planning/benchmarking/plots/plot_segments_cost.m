% This plot attempts to figure out how many segments you need vs.
% density.
% But it sucks so don't use it.
figure_path = '/Users/helen/papers/asldoc-2016-iros-oleynikova-replanning/figures/matlab/';
do_print = 1;

% Group trials by number of segments.
% Col 1 is success, col 2 is failure.
segment_costs = zeros(5, 2);
segment_successes = zeros(5, 1);
total_trials = zeros(5, 1);

% Same stats for with restarts
segment_costs_restart = zeros(5, 2);
segment_successes_restart = zeros(5, 1);
total_trials_restart = zeros(5, 1);

for i = 1:length(trials)
  % See where this fits.
  for j = 1:length(trials(i).pto_trials)
    num_segments = trials(i).pto_trials(j).params.num_segments;
    
    if (trials(i).pto_trials(j).params.random_restarts)
      total_trials_restart(num_segments) = total_trials_restart(num_segments) + 1;

      if (trials(i).pto_trials(j).metrics.success)
        segment_successes_restart(num_segments) = segment_successes_restart(num_segments) + 1;
        segment_costs_restart(num_segments, 1) = segment_costs_restart(num_segments, 1) + trials(i).pto_trials(j).metrics.final_cost;
      else
        segment_costs_restart(num_segments, 2) = segment_costs_restart(num_segments, 2) + trials(i).pto_trials(j).metrics.final_cost;
      end
    else
      total_trials(num_segments) = total_trials(num_segments) + 1;

      if (trials(i).pto_trials(j).metrics.success)
        segment_successes(num_segments) = segment_successes(num_segments) + 1;
        segment_costs(num_segments, 1) = segment_costs(num_segments, 1) + trials(i).pto_trials(j).metrics.final_cost;
      else
        segment_costs(num_segments, 2) = segment_costs(num_segments, 2) + trials(i).pto_trials(j).metrics.final_cost;
      end
    end
  end
end

% figure(4)
% clf
% Short-hand test:
mean_trials = mean(total_trials);
% 
% % This next figure is about number of segments vs overall cost. Or maybe
% % success. Who knows. Taken over all trials.
% plot(1:5, segment_costs(:, 1)./segment_successes,...
%   1:5, segment_costs(:, 2)./(total_trials-segment_successes), ...
%   1:5, segment_successes)
% 
% figure(5)
% plot(1:5, segment_costs_restart(:, 1)./segment_successes_restart, ...
%   1:5, segment_costs_restart(:, 2)./(total_trials-segment_successes_restart), ...
%   1:5, segment_successes_restart)
% % 
% 
% figure(5)
% plot(1:5, segment_successes/mean_trials, 1:5, segment_successes_restart/mean_trials);

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
% set(gca,'FontSize',12);
% if (do_print)
%   print('-dpng', [figure_path 'success_density.png']);
%   print('-depsc', [figure_path 'success_density.eps']);
% end
% hold off;


%plot(1:5, pto_success(:, :), '--x', 1:5, pto_success_restart(:, :), '-.s');

figure(6);
clf;
clear cmap;
bar(horzcat(segment_successes, segment_successes_restart)/mean_trials, 'hist')
cmap = colormap('parula(12)');
cmap2(1, :) = cmap(1, :);
cmap2(2, :) = cmap(end/2, :);
colormap(cmap2);
xlabel('Number of Segments')
ylabel('Success [{fraction}]')
legend('Without Random Restarts', 'With Random Restarts', 'Location', 'SouthEast');
title('Fraction of Successful Plans by Number of Segments');
set(gca,'FontSize',12);
if (do_print)
  print('-dpng', [figure_path 'success_segments.png']);
  print('-depsc', [figure_path 'success_segments.eps']);
end
hold off;