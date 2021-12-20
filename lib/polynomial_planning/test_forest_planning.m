% Part 1 - create a huge map!
% Todo - obstacles of varying sizes, etc.
map = create_random_map(100, 100, 10, 2000, 0.5);
show(map)
hold on;

% Part 2 - plan an initial path!
start_point = [1, 1];
goal_point = [99, 99];

while is_point_in_collision(map, start_point)
  start_point = start_point + rand(1, 2) - 0.5;
end

while is_point_in_collision(map, goal_point)
  goal_point = goal_point + rand(1, 2) - 0.5;
end

waypoints = [start_point; goal_point];

%plot(waypoints(:, 1), waypoints(:, 2), 'x');

global_plan = plan_path_waypoints(waypoints);
plot_trajectory(global_plan);


dt = 0.1;
[t, p, v] = sample_trajectory(global_plan, dt);



% Part 3 - simulate a helicopter flying through.
% First, just sample the whole trajectory.
% Go over these samples...
% Check horizon for collisions.

for i = 1:10:length(t)
  
  % Sample at the time.
  %[~, p] = sample_trajectory_at_time(global_plan, t);
  
  
  % Check the horizon (5 sec?)
  j_max = min(length(t), i + 5/dt);
  for j = i:j_max
    if (is_point_in_collision(map, p(j, :)))
      
      % Gotta replan a section!
      submap = get_submap(map, p(i, :), [4 4]);
      
      figure(2);
      show(submap);
      
      % Do astar on submap!
      figure(1);
      path = a_star(map, p(i, :) - submap.GridLocationInWorld, p(i+5/dt, :) - submap.GridLocationInWorld);
      if (isempty(path))
        break;
      end
      path(:, 1) = path(:, 1) + submap.GridLocationInWorld(1);
      path(:, 2) = path(:, 2) + submap.GridLocationInWorld(2);
      short_path = shortcut_path(map, path);
      trajectory = plan_path_waypoints(short_path);
      trajectory = split_trajectory(map, trajectory);
      plot_trajectory(trajectory);
      %disp 'Collision';
      pause
      break;
    end
  end
  
  plot(p(i, 1), p(i, 2), 'xr');
  pause(0.01)
end


% Part 4 - when the horizon is in collision, do something.

% 
% path = a_star(map, start_point, goal_point);
% if isempty(path)
%   return;
% end
% plot(path(:, 1), path(:, 2));
% 
% short_path = shortcut_path(map, path);
% trajectory = plan_path_waypoints(short_path);
% %split_trajectory;
% trajectory = split_trajectory(map, trajectory);
% 
% plot(path(:, 1), path(:, 2));
% plot_trajectory(trajectory);
% 

hold off;
