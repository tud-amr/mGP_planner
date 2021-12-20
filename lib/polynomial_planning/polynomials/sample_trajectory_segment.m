function [t, p, v, a] = sample_trajectory_segment(segment, dt)
% Derivatives are too hard. ;)
% p is [T x K] array.

K = size(segment.coefficients, 2);
segment_time = segment.time;
num_samples = round(segment_time/dt);
current_time = 0;

p = zeros(num_samples, K);
v = p;
a = p;
p_ind = 1;
t = [];

t1 = 0:dt:segment_time;
% Iterate over all dimensions.
for j = 1 : K
    % Need to reverse order. :)
    p_coeffs = flipud(segment.coefficients(:, j));
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