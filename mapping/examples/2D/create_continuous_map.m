function [ground_truth_map] = create_continuous_map(dim_x, dim_y, cluster_radius)
% Creates 2D 'farmland' environment with random distribution of % veg.
% cover.
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

visualize = 1;

% Vegetation cover range (%).
% veg_cover ∈ [veg_cover_min, veg_cover_max]
veg_cover_min = 0;
veg_cover_max = 1;

% Create map.
n_y = dim_y*2;
n_x = dim_x*2;
r = cluster_radius; % radius (maximal 4)
noise =  randn(n_y, n_x);
[ground_truth_map,y]=meshgrid(-r:r,-r:r);
mask=((ground_truth_map.^2+y.^2)<=r^2);  %(2*r+1)x(2*r+1) bit mask
ground_truth_map = zeros(n_y,n_x);
nmin_x = r+1; nmax_x = nmin_x + dim_x - 1;
nmin_y = r+1; nmax_y = nmin_y + dim_y - 1;

for i=nmin_y:nmax_y
    for j=nmin_x:nmax_x
        A = noise((i-r):(i+r), (j-r):(j+r));
        ground_truth_map(i,j) = sum(sum(A.*mask));
    end
end

Nr = sum(sum(mask));
ground_truth_map = ground_truth_map(nmin_y:nmax_y, nmin_x:nmax_x)/Nr;

% Normalize.
range = max(max(ground_truth_map)) - min(min(ground_truth_map));
ground_truth_map = (ground_truth_map - min(min(ground_truth_map))) / range;
ground_truth_map = (ground_truth_map * (veg_cover_max - veg_cover_min)) + veg_cover_min;

% Plot.
if (visualize)
    imagesc(ground_truth_map);
    set(gca,'Ydir','Normal');
    title('Ground truth map')
end

end

