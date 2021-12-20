%map = create_forest_map;
%rng(1)
%map = create_forest_map(1, 10);

start_point = [0.5 0.5];
goal_point = [4.5, 4.5];

%start_point = [9*rand() 9*rand()];
%goal_point = [9*rand() 9*rand()];

visualize = 1;

figure(1)
clf;
axis equal;
[chomp_path, chomp_metrics] = evaluate_toolbox_chomp(map, start_point, goal_point, visualize);

%%
figure(2)
clf
%plot_map_table(map, get_map_table(map));
hold on;
colormap(flipud(colormap('gray')))
[traj_out, cont_metrics] = evaluate_continuous(map, start_point, goal_point, visualize, 2, 0);

%% Compare metrics
chomp_metrics
cont_metrics