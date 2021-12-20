function path = search_lattice_viewpoints(viewpoint_init, lattice_viewpoints, ...
    faces_map, map_parameters, sensor_parameters, planning_parameters)
% Performs a greedy grid search over a list of candidates to identify
% most promising points to visit based on an informative objective.
% Starting point is fixed (no measurement taken here)
% ---
% Inputs:
%   - viewpoint_init: starting viewpoint
%   - lattice_viewpoints: list of candidates to evaluate
%   - lattice_los_neighbors: lattice los neighbors information
%   - faces_map: current faces map (mean + covariance)
% ---
% Output:
% path: grid search result
% ---
% H Zhu 2020
%

    viewpoint_prev = viewpoint_init;
    path = viewpoint_init;

    % First measurement?
    while (planning_parameters.control_points > size(path, 1))

        % Initialise best solution so far.
        viewpoint_best = find_next_best_latticepoint(viewpoint_prev, lattice_viewpoints, ...
            faces_map, map_parameters, sensor_parameters, planning_parameters);

        % Update the map with measurement at best point.
        faces_map = predict_map_var_update(viewpoint_best, faces_map, ...
            map_parameters, sensor_parameters);
        disp(['Point ', num2str(size(path,1)+1), ' at: ', num2str(viewpoint_best)]);
        disp(['Trace of P: ', num2str(trace(faces_map.P))]);
        if viewpoint_best(3) < 0
            return;
        else
            path = [path; viewpoint_best];
            viewpoint_prev = viewpoint_best;
        end

    end

end
