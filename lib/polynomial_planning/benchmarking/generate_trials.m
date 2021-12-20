map_size = 5;
tree_size = 1;
% Nominal tree density per map.
tree_density = [5:2:25];
trials_per_map = 10;

% Select start and end points (always the same per trial).
start_point = [0.5 0.5];
goal_point = [4.5, 4.5];

% Select some numbers for PTO evals.
num_segments = 1:5;
random_restarts = 0:1;

% How many restarts to do when doing restarts, and choose best.
num_restarts = 5;

% Count up and use this as the rng.
trial_number = 1;

visualize = 0;

clear trials;

for i = 1:length(tree_density)
  for j = 1:trials_per_map
   % Generate a map.
   rng(trial_number);
   map = create_forest_map(1, tree_density(i), map_size, tree_size);

   % Figure out if this is an okay map.
   feasible = verify_map_feasibility(map, start_point, goal_point, 0);
   while (~feasible)
    trial_number = trial_number + 1;
    rng(trial_number);
    map = create_forest_map(1, tree_density(i), map_size, tree_size);
    feasible = verify_map_feasibility(map, start_point, goal_point, 0);
    disp 'Refreshing map';
   end
   
   trial = struct();
   % Fill in map data.
   trial.map = struct('trial_number', trial_number, 'size', map_size, ...
     'tree_size', tree_size, 'tree_density', tree_density(i));
   
   % Run the chomp eval.
%    figure(1)
%    clf;
%    plot_map_table(map, get_map_table(map));
%    axis equal;
%    colormap(flipud(colormap('gray')));
   [chomp_path, chomp_metrics] = evaluate_toolbox_chomp(map, start_point, goal_point, visualize);

   chomp_trials = struct;
   chomp_trials(1).metrics = chomp_metrics;
   
   % Run the pto eval.
%    figure(2)
%    clf;
%    plot_map_table(map, get_map_table(map));
%    axis equal;
%    hold on;
%    colormap(flipud(colormap('gray')));
   pto_index = 1;
   
   clear pto_trials;
   pto_trials = create_pto_struct;
   for k = 1:length(num_segments)
     for l = 1:length(random_restarts)
        pto_trials(pto_index) = create_pto_struct;
        pto_trials(pto_index).params.num_segments = num_segments(k);
        pto_trials(pto_index).params.random_restarts = random_restarts(l);
       
       if (~random_restarts(l))
        [traj_out, cont_metrics] = evaluate_continuous(map, start_point, ...
          goal_point, visualize, num_segments(k), 0);
       else
         for restart_index = 1:num_restarts
           [traj_out, restart_metrics] = evaluate_continuous(map, start_point, ...
              goal_point, visualize, num_segments(k), 1);
            % Only works if we ran without random restarts first, but
            % that's fine.
            if (restart_metrics.final_cost < cont_metrics.final_cost)
              cont_metrics = restart_metrics;
            end
         end
       end
       pto_trials(pto_index).metrics = cont_metrics;
       pto_index = pto_index + 1;
     end
   end

   trial.chomp_trials = chomp_trials;
   trial.pto_trials = pto_trials;
   if (~exist('trials'))
     trials = trial;
   else
    trials(end+1) = trial;
   end

   trial_number = trial_number + 1;
   %drawnow;
   %pause(0.01);
  end
end