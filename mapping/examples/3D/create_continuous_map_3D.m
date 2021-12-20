function [ground_truth_map] = create_continuous_map_3D(dim_x, dim_y, dim_z, cluster_radius)
% Creates 3D environment with random distribution of information.
%
% Coordinate system:
%
%  ^ y (rows)
%  |
%  |
%  -----> x (cols)
%
% Adapted from:
% https://people.smp.uq.edu.au/DirkKroese/ps/MCSpatial.pdf

% Vegetation cover range (%).
% veg_cover ∈ [veg_cover_min, veg_cover_max]
veg_cover_min = 0;
veg_cover_max = 1;

% Create map.
n_z = dim_z*2;
n_y = dim_y*2;
n_x = dim_x*2;
r = cluster_radius; % radius (maximal 4)
noise =  randn(n_y,n_x,n_z);
[ground_truth_map,y,z]=meshgrid(-r:r,-r:r,-r:r);
mask=((ground_truth_map.^2+y.^2+z.^2)<=r^2);  %(2*r+1)x(2*r+1) bit mask
ground_truth_map = zeros(n_y,n_x,n_z);
nmin_x = r+1; nmax_x = nmin_x + dim_x - 1;
nmin_y = r+1; nmax_y = nmin_y + dim_y - 1;
nmin_z = r+1; nmax_z = nmin_z + dim_z - 1;

for i=nmin_y:nmax_y
    for j=nmin_x:nmax_x
        for k=nmin_z:nmax_z
            A = noise((i-r):(i+r), (j-r):(j+r), (k-r):(k+r));
            ground_truth_map(i,j,k) = sum(sum(sum(A.*mask)));
        end
    end
end

Nr = sum(sum(sum(mask)));
ground_truth_map = ground_truth_map(nmin_y:nmax_y, nmin_x:nmax_x, nmin_z:nmax_z)/Nr;

% Normalize.
range = max(max(max(ground_truth_map))) - min(min(min(ground_truth_map)));
ground_truth_map = (ground_truth_map - min(min(min(ground_truth_map)))) / range;
ground_truth_map = (ground_truth_map * (veg_cover_max - veg_cover_min)) + veg_cover_min;

end

