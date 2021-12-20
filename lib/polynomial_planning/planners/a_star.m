function [path] = a_star(map, start_point, goal_point)
%A_STAR Summary of this function goes here
%   Detailed explanation goes here
path = [];
% Create a map from the binary occupancy grid.
msg = rosmessage('nav_msgs/OccupancyGrid');
writeBinaryOccupancyGrid(msg, map);
obstacle_map = reshape(msg.Data, map.GridSize)';
map_size = size(obstacle_map);

% Convert start and goal into grid points.
res = map.Resolution;
%start = world2grid(map, start_point);
%goal = world2grid(map, goal_point);

start = flipud(round(res * start_point));
goal = flipud(round(res * goal_point));
start_ind = sub2ind(map_size, start(1), start(2));
goal_ind = sub2ind(map_size,  goal(1), goal(2));

if (obstacle_map(start_ind) > 0 || obstacle_map(goal_ind) > 0)
  disp 'Cant compute, goal or start are occupied';
  return;
end

% Create a heuristic map.
h_map = zeros(map_size);
for i=1:size(h_map, 1)
  for j=1:size(h_map, 2)
    % Faster with norm or not?
    h_map(i, j) = sqrt((i-goal(1))^2+(j-goal(2))^2);
  end
end

% This is the actual cost-to-go map.
% Initialized with -1 for unknown cells.
g_map = -ones(map_size);
g_map(start_ind) = 0;

% This will point to the sub2ind index of the parent.
parent_map = zeros(map_size);

% open and closed are stored as sub2ind.
closed_set = [];
open_set = [start_ind]; % This will be the start node.

n_iter = 0;
max_iter = 5000;

goal_found = 0;
while ~isempty(open_set)
  n_iter = n_iter + 1;

  % Get all nodes in the open set, find the minimum g + h
  [~, min_index] = min(g_map(open_set) + h_map(open_set));
  % Get the current node actual index.
  current_ind = open_set(min_index);
  
  % Remove min_index from open set.
  open_set = open_set([1:min_index-1, min_index+1:end]);
  
  % But add the current index to the closed set.
  closed_set(end+1) = current_ind;
  
  % Check if this is the goal node, first of all.
  if current_ind == goal_ind
    %disp 'FOUND GOAL!!!';
    goal_found = 1;
    break;
  end
  
  % Get the neighbors.
  neighbor_inds = get_neighbors(map_size, current_ind);
  for i = 1:length(neighbor_inds)
    ind = neighbor_inds(i);
    
    % Check if this is occupied.
    if obstacle_map(ind) > 0
      continue;
    end
    
    % Check if we've already evaluated this.
    if ~isempty(find(closed_set == ind, 1))
      continue;
    end
    
    % Compute the g score from this node.
    g = g_map(current_ind) + get_distance(map_size, ind, current_ind);
    
    % Check if this is a new node or the g cost is better than the last.
    if (g_map(ind) < 0 || g_map(ind) > g)
      g_map(ind) = g;
      parent_map(ind) = current_ind;
      if isempty(find(open_set == ind, 1))
        open_set(end+1) = ind;
      end
    end
  end
end

% Go backwards to reconstruct the path.
if ~goal_found
  return;
end

current_ind = goal_ind;
path_inds = [];
while current_ind ~= start_ind
  path_inds(end+1) = current_ind;
  current_ind = parent_map(current_ind);
end

[path_row, path_col] = ind2sub(map_size, path_inds);

path = horzcat((path_col' - 0.5)/res, (path_row' - 0.5)/res);

% Reverse order! Whoops.
path = flipud(path);

% close all;
% clf;
% figure(1);
% pcolor(g_map);
% colormap gray;
% hold on;
% plot(path_col + 0.5, path_row + 0.5);
% hold off;
% figure(2);
% show(map)
% hold on;
% plot(path(:, 1), path(:, 2));
% hold off;
% figure(3);
% pcolor(obstacle_map);
% figure(4);
% pcolor(h_map);

% Solution found?
end

function Y = get_neighbors(map_size, current_ind)
  [r,c] = ind2sub(map_size, current_ind);

  % Of 8-way neighbors, only use those w/in grid bounds
  Y = [r-1 r-1 r-1 r r  r+1 r+1 r+1; c-1 c c+1 c-1 c+1 c-1 c c+1];
  %Y = [r-1 r r r+1; c c+1 c-1 c];
  k = (min(Y)>0) & (Y(1,:)<=map_size(1)) & (Y(2,:)<=map_size(2));
  Y = Y(:,k);
  Y = sub2ind(map_size, Y(1,:)', Y(2,:)')';
end

function dist = get_distance(map_size, ind1, ind2)
  [pos1] = ind2sub(map_size, ind1);
  [pos2] = ind2sub(map_size, ind2);
  
  dist = norm(pos2 - pos1);
end
