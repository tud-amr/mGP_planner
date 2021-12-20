function [path_optimized] = optimize_with_cmaes_inspect(path, faces_map, map_parameters, ...
    sensor_parameters, planning_parameters, optimization_parameters)
% Optimizes a polynomial path (defined by control points) using Covariance
% Matrix Adaptation Evolutionary Strategy (CMA-ES).
% ---
% H Zhu 2020
%

    % Set optimization parameters.
    opt = cmaes;
    opt.DispFinal = 'off';
    opt.LogModulo = 0;
    opt.TolFun = 1e-9;
    opt.IncPopSize = 1; %% Check this
%     opt.PopSize = 25;
    opt.SaveVariables = 'off';
    opt.MaxIter = optimization_parameters.max_iters;
    opt.Seed = randi(2^10);

    % Set bounds and covariances.
    dim_x_env = map_parameters.dim_x_env;
    dim_y_env = map_parameters.dim_y_env;
    dim_z_env = map_parameters.dim_z_env;
    LBounds = [dim_x_env(1); dim_y_env(1); dim_z_env(1); -pi];
    UBounds = [dim_x_env(2); dim_y_env(2); dim_z_env(2);  pi];
    cov = [optimization_parameters.cov_x; 
       optimization_parameters.cov_y; 
       optimization_parameters.cov_z;
       optimization_parameters.cov_yaw];
    if (optimization_parameters.opt_yaw)    % optimize yaw
        num_opt = 4;
    else
        num_opt = 3;
    end
    LBounds = LBounds(1:num_opt);
    UBounds = UBounds(1:num_opt);
    cov = cov(1:num_opt);
    opt.LBounds = repmat(LBounds, size(path,1)-1, 1);
    opt.UBounds = repmat(UBounds, size(path,1)-1, 1);
    cov = repmat(cov, size(path,1)-1, 1);

    % Remove starting point (as this is fixed).
    
    path_initial = reshape(path(2:end,1:num_opt)', [], 1);
    path_optimized = cmaes('optimize_viewpoints', path_initial, cov, opt, path(1,1:num_opt), ...
        faces_map, map_parameters, sensor_parameters, planning_parameters, optimization_parameters);
    path_optimized = reshape(path_optimized, num_opt, [])';
    path_optimized = [path(1,1:num_opt); path_optimized];

end
