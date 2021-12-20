function faces_map = predict_map_var_update(viewpoint, faces_map, ...
    map_parameters, sensor_parameters)

    %% cam pose
    cam_pos = viewpoint(1:3)';
    cam_roll = sensor_parameters.cam_roll;
    cam_pitch = sensor_parameters.cam_pitch;
    cam_yaw = sensor_parameters.cam_yaw + viewpoint(4);

    %% cam fov
    [A, b] = get_camera_fov_constraints(cam_pos, cam_roll, cam_pitch, cam_yaw, ...
        sensor_parameters.fov_x, sensor_parameters.fov_y, ...
        sensor_parameters.fov_range_min, sensor_parameters.fov_range_max);
    
    %% determine currently observed faces and fetch measurements
    faces_observed = [];
    faces_submap_var = [];
    for i = 1 : map_parameters.num_faces
        in = 1;
        % face range and incidence
        [range, incidence] = get_incidence_range(...
            map_parameters.F_points(i,:,:), map_parameters.F_center(i,:)', ...
            map_parameters.F_normal(i,:)', cam_pos);
        % determine if observed
        if range > sensor_parameters.fov_range_max || ...
                range < sensor_parameters.fov_range_min
            in = 0;
            continue;
        end
        if incidence < sensor_parameters.incidence_range_min
            in = 0;
            continue;
        end
        if sum(A*map_parameters.F_center(i,:)' > b) > 0      % vialting any one
            in = 0;
            continue;
        end
        % fetch information if observed
        if (in)
            % face index
            faces_observed = [faces_observed; i];
            % observation noise
            var = sensor_model_inspect(range, incidence, sensor_parameters);
            faces_submap_var = [faces_submap_var; var];
        end
    end
    
    %% update map variance
    % observation matrix construction
    num_faces = map_parameters.num_faces;               % n
    num_faces_observed = length(faces_observed);        % m 
    H = zeros(num_faces_observed, num_faces);           % mxn
    for i = 1 : num_faces_observed
        idx = faces_observed(i);
        H(i, idx) = 1;
    end
    % observation and noise
    R = diag(faces_submap_var);
    % KF filter
    faces_map.P = KF_update_cholesky(faces_map.P, R, H);

end