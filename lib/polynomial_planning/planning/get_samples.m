function samples = get_samples(center_point, path_direction_vector, sample_radius, num_samples)
%GET_SAMPLES
% Pre-alloc the samples vector.
samples = zeros(num_samples*2 + 1, 2);

% Only true for 2D... gets more complicated in 3D.
normal_rotation = [0 -1; 1 0];

path_normal_vector = (normal_rotation * path_direction_vector')';

increment_size = sample_radius/num_samples;

% First the positive direction.
for i = 1:num_samples
  samples(i, :) = center_point + path_normal_vector * increment_size * i;
end

for j = 1:num_samples
  samples(i+j, :) = center_point - path_normal_vector * increment_size * j;
end

samples(i+j+1, :) = center_point;
end

