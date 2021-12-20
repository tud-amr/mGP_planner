function viewpoint_best = find_next_best_latticepoint(viewpoint_prev, lattice_viewpoints, ...
    faces_map, map_parameters, sensor_parameters, planning_parameters)
% Find the next best lattice viewpoint via greedy search

    P_trace_prev = trace(faces_map.P);

    % Initialise best solution so far.
    obj_min = Inf;

    for i = 1:size(lattice_viewpoints, 1)

        viewpoint_eval = lattice_viewpoints(i, :);

        % if the viewpoint is in LoS && within some range
        if (if_in_los(viewpoint_prev(1:3)', viewpoint_eval(1:3)', map_parameters) && ...
                norm(viewpoint_prev(1:3)'-viewpoint_eval(1:3)') <= map_parameters.lattice_range)
            faces_map_eval =  predict_map_var_update(viewpoint_eval, faces_map, ...
                map_parameters, sensor_parameters);
            P_trace = trace(faces_map_eval.P);

            gain = P_trace_prev - P_trace;

            if (strcmp(planning_parameters.obj, 'exponential'))
                cost = pdist([viewpoint_prev(1:3); viewpoint_eval(1:3)])/planning_parameters.max_vel;
                obj = -gain*exp(-planning_parameters.lambda*cost);
            elseif (strcmp(planning_parameters.obj, 'rate'))
                cost = max(pdist([viewpoint_prev(1:3); viewpoint_eval(1:3)])/planning_parameters.max_vel, ...
                    1/planning_parameters.measurement_frequency);
                obj = -gain/cost;
            end

            %disp(['Point ', num2str(viewpoint_eval)]);
            %disp(['Gain: ', num2str(gain)])
            %disp(['Cost: ', num2str(cost)])
            %disp(num2str(obj));

            % Update best solution.
            if (obj < obj_min)
                obj_min = obj;
                viewpoint_best = viewpoint_eval;
            end
            
%             if (obj_min >= 0)       % no infomation gain
%                 disp('Could not find the best lattice point! Planning is complete.');
%                 viewpoint_best = [1000, 1000, -10, 0];
%             end
            
        else
            continue;
        end

    end

    disp(['Objective: ', num2str(obj_min)]);

end
