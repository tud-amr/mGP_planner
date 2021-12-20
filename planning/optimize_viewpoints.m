function obj = optimize_viewpoints(waypoints, starting_point, faces_map, ...
     map_parameters, sensor_parameters, planning_parameters, optimization_parameters)
% Fitness function for optimizing all points on a horizon for an informative 
% objective

if (optimization_parameters.opt_yaw)    % optimize yaw
    num_opt = 4;
else
    num_opt = 3;
end

waypoints = reshape(waypoints, num_opt, [])';
waypoints = [starting_point; waypoints];

obj = compute_objective_inspect(waypoints, faces_map, map_parameters,...
    sensor_parameters, planning_parameters, optimization_parameters);

end