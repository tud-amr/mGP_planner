map = create_map();

%% Sample a polynomial path through waypoints.
%waypoints = [4 0.5; 1 3; 3 5; 7 7; 2 3];
%waypoints = [0.5, 1.0; 2 2; 6, 5; 5 9; 1 5];
% waypoints = [0.5, 1.0; 2 1.5; 1 4.5]; % Failure case.

%waypoints = [0.5, 1.0; 2 1.5; 6.5, 2.5]; % Bigger tree level.
waypoints = [0.5, 1.0; 2 1.5; 7.5, 2.5]; % Works nicely with tree level 1.

waypoints = [0.5, 1.0; 2 1.5; 9, 4]; % Works nicely with tree level 1.

trajectory = plan_path_waypoints(waypoints);
[t, p] = sample_trajectory(trajectory, 0.1);

[colliding_times, colliding_segments, colliding_p] = is_trajectory_in_collision(map, trajectory);

show(map);
hold on;
%comet(p(:, 1), p(:, 2));
plot(p(:, 1), p(:, 2), 'LineWidth', 2);
plot(waypoints(:, 1), waypoints(:, 2), 'x');
plot(colliding_p(:, 1), colliding_p(:, 2), 'o');
hold off;

%% Replan segments in collision.
start_point =  waypoints(2, :);
goal_point = waypoints(3, :);
%replan_segment
replan_segment(map, waypoints(2, :), waypoints(3, :));

%% Sample a PRM path through waypoints.
% start = [0.5, 1.0];
% goal = [1.0, 5.0];
% 
% prm = robotics.PRM;
% prm.Map = map;
% prm.NumNodes = 50;
% prm.ConnectionDistance = 5;
% tic
% path = findpath(prm, start, goal);
% toc
% show(prm)

%% Plan a polynomial path through the waypoints
% trajectory = plan_path_waypoints(path);
% [t, p] = sample_trajectory(trajectory, 0.1);
% 
% show(map);
% hold on;
% plot(p(:, 1), p(:, 2), 'LineWidth', 2);
% plot(path(:, 1), path(:, 2), 'x');
% hold off;
