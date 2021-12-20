function [colliding_times, colliding_segments, colliding_p] = is_trajectory_in_collision(map, trajectory)
%IS_TRAJECTORY_IN_COLLISION Summary of this function goes here
%   Detailed explanation goes here
if (nargin < 3)
  min_segment = 1;
  max_segment = trajectory.num_elements;
end

% Only need to sample every... 0.1? 0.2? who cares.
checking_resolution = 0.2;
  
[t, p] = sample_trajectory(trajectory, checking_resolution);

colliding_times = [];
colliding_segments = [];
colliding_p = [];
for i = 1:length(t)
  if is_point_in_collision(map, p(i, :))
    colliding_times(end+1) = t(i);
    colliding_p(end+1, :) = p(i, :);
  end
end

% Figure out which segment the t corresponded to.
% t is monotonically increasing (yay) so this is quite easy.
current_time = 0;
current_segment = 1;
for i = 1:length(colliding_times)
  if colliding_times(i) <= current_time
    colliding_segments(i) = current_segment - 1;
  else
    % Move forward the index of the segments until we find a match.
    while colliding_times(i) > current_time
      current_time = current_time + trajectory.segments(current_segment).time;
      current_segment = current_segment + 1;
      if (current_segment >= trajectory.num_elements)
        break
      end
    end
    colliding_segments(i) = current_segment - 1;
    if (current_segment >= trajectory.num_elements)
      break
    end
  end
end
end

