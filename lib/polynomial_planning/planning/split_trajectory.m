function trajectory = split_trajectory(map, trajectory)
%SPLIT_TRAJECTORY Summary of this function goes here
%   Detailed explanation goes here

max_splits = 5;

[colliding_times, colliding_segments] = is_trajectory_in_collision(map, trajectory);

if isempty(colliding_times)
  return;
end

% Store all existing vertices.
num_vertices = trajectory.num_elements;
vertices = [];
for i = 1:num_vertices
  % Hmm at some point meet start and end constraints as well...
  % Is this working???
  [vertices(i, 1:2), vertices(i, 3:4), vertices(i, 5:6)] = get_vertex_trajectory(trajectory, i);
end

% Fill in start and end derivatives.
start_derivatives = [vertices(1, 3:4) ; vertices(1, 5:6)];
end_derivatives = [vertices(end, 3:4) ; vertices(end, 5:6)];

for i = 1:max_splits
  if isempty(colliding_times)
    break;
  end
  
  % Store all segments start times.
  segment_start_time = zeros(size(trajectory.segments));
  total_time = 0;
  for i = 1 : length(trajectory.segments)
    segment_start_time(i) = total_time;
    total_time = total_time + trajectory.segments(i).time;
  end
  
  % Figure out which segment are in collision.
  segments_to_split = unique(colliding_segments);
  
  for j = 1:length(segments_to_split)
    % Split the first position in the segment.
    segment_index = segments_to_split(j);
    k = find(colliding_segments == segment_index, 1);
    relative_split_time = (colliding_times(k) - ...
      segment_start_time(segment_index))/trajectory.segments(segment_index).time;
    % Find the new position.
    if (relative_split_time < 0.25)
      relative_split_time = 0.25;
    elseif (relative_split_time > 0.75)
      relative_split_time = 0.75;
    end
    split_pos = vertices(segment_index, 1:2) + relative_split_time*(vertices(segment_index + 1, 1:2) - vertices(segment_index, 1:2));
    
    % Uhh have to figure this out.
    vertices(segment_index+2:end+1, :) = vertices(segment_index+1:end, :);
    vertices(segment_index+1, 1:2) = split_pos;    
  end
  trajectory = plan_path_waypoints(vertices, start_derivatives, end_derivatives);
  [colliding_times, colliding_segments] = is_trajectory_in_collision(map, trajectory);
end

end 



