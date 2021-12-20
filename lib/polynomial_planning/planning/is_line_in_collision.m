function [colliding_p] = is_line_in_collision(map, start_point, goal_point)
%IS_TRAJECTORY_IN_COLLISION Summary of this function goes here
%   Detailed explanation goes here

% Only need to sample every... 0.1? 0.2? who cares.
checking_resolution = 0.02;

% Reduce to a 1D problem
distance = norm(goal_point - start_point);
direction_vector = (goal_point - start_point)/distance;

% How many points are there?
num_points = floor(distance/checking_resolution) + 1;

colliding_p = [];

% We stick the last point in there anyway.
point = start_point - direction_vector*checking_resolution;
for i = 1 : num_points
  point = point + direction_vector*checking_resolution;
  if i == num_points
    point = goal_point;
  end
  if is_point_in_collision(map, point)
    colliding_p(end+1, :) = point;
  end
end
end

