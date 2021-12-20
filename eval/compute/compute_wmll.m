function wmll = compute_wmll(grid_map, ground_truth_map)
% p.23 of http://www.gaussianprocess.org/gpml/chapters/RW2.pdf
% Bayesian Opt. for Informative Continuous Path Planning - Marchant & Ramos (ICRA, 2014)

weights = (ground_truth_map - min(min(ground_truth_map))) / ...
    (max(max(ground_truth_map)) - min(min(ground_truth_map)));
weights = weights/sum(sum(weights));

P = reshape(diag(grid_map.P)', size(grid_map.m));

wmll = weights.*((0.5.*log(2.*pi.*P)) + ...
    (((ground_truth_map - grid_map.m).^2)./(2.*P)));
wmll = sum(sum(wmll));

end
