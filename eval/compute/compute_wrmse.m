function wrmse = compute_wrmse(grid_map_mean, ground_truth_map)
% Computes the WRMSE of a map wrt. ground truth.
% Bayesian Opt. for Informative Continuous Path Planning - Marchant & Ramos (ICRA, 2014)

weights = (ground_truth_map - min(min(ground_truth_map))) / ...
    (max(max(ground_truth_map)) - min(min(ground_truth_map)));
weights = weights/sum(sum(weights));
%weights = (weights - min(min(weights))) / ...
%    (max(max(weights)) - min(min(weights)));

wse = sum(sum(weights.*((grid_map_mean - ground_truth_map).^2)));
%wmse = wse / numel(ground_truth_map);
wrmse = sqrt(wse);

end