function mll = compute_mll(grid_map, ground_truth_map)
% p.23 of http://www.gaussianprocess.org/gpml/chapters/RW2.pdf
% Bayesian Opt. for Informative Continuous Path Planning - Marchant & Ramos (ICRA, 2014)

P = reshape(diag(grid_map.P)', size(grid_map.m));
    
ll = (0.5.*log(2.*pi.*P)) + ...
    (((ground_truth_map - grid_map.m).^2)./(2.*P));
ll_sum = sum(sum(ll));
mll = ll_sum/numel(grid_map.m);

end