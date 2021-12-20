%% Create a random map.
map_size = 1;

if (map_size == 2)
  map = create_random_map(10, 10, 10, 50);
  start_point = [0.5 0.5];
  goal_point = [9.5 9.5];
elseif (map_size == 3)
  map = create_random_map(2, 2, 10, 10);
  start_point = [0.5 0.5];
  goal_point = [1.5 1.5];
else
  map = create_random_map(4, 4, 10, 10, 0.4);
  start_point = [0.5 0.5];
  goal_point = [3.5 3.5];
end


%% Clear 0.5, 0.5 and 3.5, 3.5

setOccupancy(map, vertcat(start_point, goal_point, ...
  start_point+0.05, goal_point+0.05, start_point-0.05, goal_point-0.05), 0);

show(map);
hold on;
plot([start_point(1), goal_point(1)], [start_point(2), goal_point(2)], 'x');


%% Make a plan using A*.
path = a_star(map, start_point, goal_point);
if isempty(path)
  return;
end
plot(path(:, 1), path(:, 2));

%% Approaches
% %% Plan some polynomials through this guy.
% path_length = size(path, 1);
% 
% i_max = 0;
% 
% for i = 1 : i_max
%   sampled_points = [];
%   for j = 1 : i
%     sampled_points(j, :) = path(round(j/(i+1)*path_length), :);
%   end
%   waypoints = [start_point; sampled_points; goal_point];
% 
%   trajectory = plan_path_waypoints(waypoints);
%   plot(waypoints(:, 1), waypoints(:, 2), 'o');
%   [t, p] = sample_trajectory(trajectory, 0.1);
% 
%   plot(p(:, 1), p(:, 2));
% end
% 
% %% Alternative 2 - binary splits of segments.
% waypoints = [start_point; goal_point];
% point_times = [0; 1];
%  
% trajectory = plan_path_waypoints(waypoints);
% in_collision = 1;
%  
% num_iter = 0;
% max_iter = 0;
% 
% while in_collision && num_iter < max_iter
%   num_iter = num_iter + 1;
%   [colliding_times, colliding_segments, colliding_p] = is_trajectory_in_collision(map, trajectory);
%   if isempty(colliding_times)
%     in_collision = 0;
%     disp 'Finally not in collision!';
%     break;
%   end
%   
%   
%   % Replan a new trajectory, inserting a waypoint.
%   % Assume time on path == time on straight-line path. This is a STRONG
%   % assumption...
%   new_point_time = (point_times(colliding_segments(1)) + point_times(colliding_segments(1)+1)) / 2.0;
%   i = colliding_segments(1);
%   while (min(abs(point_times - new_point_time)) < 0.05) && i <= length(point_times)-1;
%      new_point_time = (point_times(i) + point_times(i+1)) / 2.0;
%      i = i + 1;
%   end
%   
%   if min(abs(point_times - new_point_time)) < 0.01
%     disp 'Cant find a solution. :(';
%     break;
%   end
%   
%   point_times(end+1) = new_point_time;
%   point_times = sort(point_times);
%   %middle_points(end+1, :) = path(round(new_point_time*path_length), :);
%   waypoints = path(round(point_times*(path_length-1)+1), :);
%   trajectory = plan_path_waypoints(waypoints);
%   
%   [t, p] = sample_trajectory(trajectory, 0.1);
%   clf;
%   show(map);
%   hold on;
%   plot(path(:, 1), path(:, 2));
%   plot(p(:, 1), p(:, 2));
%   plot(waypoints(:, 1), waypoints(:, 2), 'x');
% end

  
%% Alternative 3 -- shortcut path, plan through it.
short_path = shortcut_path(map, path);
trajectory = plan_path_waypoints(short_path);
%split_trajectory;
trajectory = split_trajectory(map, trajectory);
[t, p] = sample_trajectory(trajectory, 0.1);
clf;
show(map);
hold on;
plot(path(:, 1), path(:, 2));

plot(short_path(:, 1), short_path(:, 2));
plot(p(:, 1), p(:, 2));

%% Alternative -- plan a polynomial until it hits a collision -- DOES NOT WORK AT THE MOMENT.
% waypoints = [start_point; goal_point];
% point_times = [0; 1];
% 
% trajectory = plan_path_waypoints(waypoints);
% in_collision = 1;
% 
% num_iter = 0;
% max_iter = 20;
% 
% while in_collision && num_iter < max_iter
%   num_iter = num_iter + 1;
%   [colliding_times, colliding_segments, colliding_p] = is_trajectory_in_collision(map, trajectory);
%   if isempty(colliding_times)
%     in_collision = 0;
%     disp 'Finally not in collision!';
%     break;
%   end
%   
%   % Replan a new trajectory, inserting a waypoint.
%   % Assume time on path == time on straight-line path. This is a STRONG
%   % assumption...
%   new_point_time = colliding_times(1) / get_trajectory_total_time(trajectory);
%   i = 2;
%   while (min(point_times - new_point_time) < 0.05) && i <= length(colliding_times)
%     new_point_time = colliding_times(i) / get_trajectory_total_time(trajectory);
%     i = i + 5;
%   end
%   point_times(end+1) = new_point_time;
%   point_times = sort(point_times)
%   %middle_points(end+1, :) = path(round(new_point_time*path_length), :);
%   waypoints = path(round(point_times*(path_length-1)+1), :)
%   trajectory = plan_path_waypoints(waypoints);
%   
%   [t, p] = sample_trajectory(trajectory, 0.1);
%   plot(p(:, 1), p(:, 2));
%   pause 
% end
% 
  
%% Replan trajectory using deterministic sampling.
%replan_segment(map, start_point, goal_point);

hold off;
