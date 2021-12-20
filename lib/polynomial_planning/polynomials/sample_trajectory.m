function [t, p, v, a] = sample_trajectory(trajectory, dt)
% Derivatives are too hard. ;)
% p is [T x K] array.

total_time = 0;
for i = 1 : length(trajectory.segments)
  total_time = total_time + trajectory.segments(i).time;
end
num_samples = round(total_time/dt);
current_time = 0;

p = zeros(num_samples, trajectory.K);
v = p;
a = p;
p_ind = 1;
t = [];
% Iterate over all segments.
for i = 1 : length(trajectory.segments)
    
  segment_time = trajectory.segments(i).time;
    
  if (i == 1)
     t1 = 0:dt:segment_time;
  elseif (dt-(current_time-t(end)) < dt)
     t1 = dt-(current_time-t(end)):dt:segment_time;
  else
     continue;
  end
  
  % Append the last sample again? Might be between samples. :/
  %t1 = [t1 segment_time];
  % Iterate over all dimensions.
  for j = 1 : trajectory.K
    % Need to reverse order. :)
    p_coeffs = flipud(trajectory.segments(i).coefficients(:, j));
    v_coeffs = polyder(p_coeffs);
    a_coeffs = polyder(v_coeffs);
    pos1 = polyval(p_coeffs, t1);
    vel1 = polyval(v_coeffs, t1);
    accel1 = polyval(a_coeffs, t1);
    p(p_ind:p_ind + size(pos1, 2)-1, j) = pos1;
    v(p_ind:p_ind + size(pos1, 2)-1, j) = vel1;
    a(p_ind:p_ind + size(pos1, 2)-1, j) = accel1;
  end
  t = [t, t1+current_time];
  current_time = current_time + segment_time;
  p_ind = p_ind + length(t1);
end

end