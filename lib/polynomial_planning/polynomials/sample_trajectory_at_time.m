function [t, p, v, a] = sample_trajectory_at_time(trajectory, t)
% Derivatives are too hard. ;)
% p is [T x K] array.

current_time = 0;

p = zeros(1, trajectory.K);
v = p;
a = p;
% Iterate over all segments.
for i = 1 : length(trajectory.segments)
  segment_time = trajectory.segments(i).time;
  
  if current_time + segment_time > t
    % Then our princess is in this castle! :)
    t_in_segment = t - current_time;

    for j = 1 : trajectory.K
      p_coeffs = flipud(trajectory.segments(i).coefficients(:, j));
      v_coeffs = polyder(p_coeffs);
      a_coeffs = polyder(v_coeffs);
      p(j) = polyval(p_coeffs, t_in_segment);
      v(j) = polyval(v_coeffs, t_in_segment);
      a(j) = polyval(a_coeffs, t_in_segment);
    end
    break;
  end
  
  % Otherwise just keep going.
  current_time = current_time + segment_time;
end

end