function [ metrics ] = create_metrics_struct()
metrics = struct();
metrics.success = 0;
metrics.length = 0;
metrics.sum_of_squared_snaps = 0;
metrics.num_iter = 0;
metrics.final_cost = 0;
end

