function path = random_select_lattice_viewpoints(viewpoint_init, lattice_viewpoints, ...
    faces_map, map_parameters, sensor_parameters, planning_parameters)
% Random select viewpoints from a list of candidates to identify
% ---
% Inputs:
%   - viewpoint_init: starting viewpoint
%   - lattice_viewpoints: list of candidates to evaluate
%   - lattice_los_neighbors: lattice los neighbors information
%   - faces_map: current faces map (mean + covariance)
% ---
% Output:
% path: viewpoint result
% ---
% H Zhu 2020
%

    viewpoint_prev = viewpoint_init;
    path = viewpoint_init;

    % Random select viewpoint one by one
    while (planning_parameters.control_points > size(path, 1))

        % first find the set of viewpoints los with current one and within
        % some range
        list_index = [];
        for i = 1:size(lattice_viewpoints, 1)
            viewpoint_eval = lattice_viewpoints(i, :);
            % if the viewpoint is in LoS && within some range
            if (if_in_los(viewpoint_prev(1:3)', viewpoint_eval(1:3)', map_parameters) && ...
                    norm(viewpoint_prev(1:3)'-viewpoint_eval(1:3)') <= map_parameters.lattice_range)
                list_index = [list_index; i];
            end
        end
        
        % random select a viewpoint from the list
        index_next = list_index(randi(length(list_index)));
        viewpoint_next = lattice_viewpoints(index_next, :);
        
        % insert in path
        path = [path; viewpoint_next];
        viewpoint_prev = viewpoint_next;
    end
    
end
