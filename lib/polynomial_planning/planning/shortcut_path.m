function waypoints = shortcut_path(map, path)
  % Find longest straight line (or feasible lengths) between points.
  
  % Naive solution first -- just check every connection? 
  % Or do the OMPL solution -- try random pairs of points.
  
  path_length = size(path, 1);
  
  
  % Shorten the path first.
  j = 1;
  waypoints = [path(j, :)];
  for i = 2 : path_length
    collision = is_line_in_collision(map, path(j, :), path(i, :));
    if (collision)
      % Then add this as a waypoint.
      waypoints(end+1, :) = path(i-1, :);
      j = i-1;
    end
    % Otherwise just keep going.
  end
  waypoints(end+1, :) = path(path_length, :);
  
  % TODO: Then go through the waypoints and shorten the waypoints.
end