function colliding = is_point_in_collision(map, point)
% Check for bounds. If out of bounds, collding.
x_range = map.XWorldLimits;
y_range = map.YWorldLimits;

if (point(1) < x_range(1) || point(1) > x_range(2))
  colliding = true;
  return;
elseif (point(2) < y_range(1) || point(2) > y_range(2))
  colliding = true;
  return;
end

colliding = getOccupancy(map, point);

end

