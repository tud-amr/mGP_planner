function [ pto_trial ] = create_pto_struct()
pto_trial.params = struct('num_segments', 0, 'random_restarts', 0);
pto_trial.metrics = create_metrics_struct();
end

