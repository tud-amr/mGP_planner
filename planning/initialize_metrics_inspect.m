function metrics = initialize_metrics_inspect()

    metrics = struct;
    metrics.path_travelled = [];
    metrics.trajectory_travelled = [];
    metrics.viewpoints_meas = [];
    metrics.faces_map_m = [];
    metrics.faces_map_P_diag = [];
    metrics.P_traces = [];
    metrics.times = [];
    metrics.rmses = [];
    metrics.wrmses = [];
    metrics.mlls = [];
    metrics.wmlls = [];

end

