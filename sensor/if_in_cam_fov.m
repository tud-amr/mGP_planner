function in = if_in_cam_fov(...
    face_points, face_center, face_normal, ...
    cam_pos, cam_roll, cam_pitch, cam_yaw, ...
    sensor_parameters)
% Determine if the triangle facet is in the camera's FOV
%
% Input:
%   face_points: points of the facet triangle, [3x3], m
%   face_center: center of the facet triangle, [3x1], m
%   face_normal: normal of the facet triangle, [3x1], m
%   cam_pos: camera position, [3x1], m
%   cam_roll: camera roll, rad
%   cam_pitch: camera pitch, rad
%   cam_yaw: camera yaw, rad
%   sensor_parameters: including camera FOV
% ---
% Output:
%   in: if the triangle facet is in the camera's FOV

    in = 1;

    % sensor parameters
    fov_x = sensor_parameters.fov_x;
    fov_y = sensor_parameters.fov_y;
    fov_range_min = sensor_parameters.fov_range_min;
    fov_range_max = sensor_parameters.fov_range_max;
    incidence_range_min = sensor_parameters.incidence_range_min;
    
    % first determine if in the range of distance and incidence
    [range, incidence_cos] = get_incidence_range(...
        face_points, face_center, face_normal, cam_pos);
    if range > fov_range_max || range < fov_range_min
        in = 0;
        return;
    end
    if incidence_cos < incidence_range_min
        in = 0;
        return;
    end
    
    % then determine if in the fov
    [A, b] = get_camera_fov_constraints(cam_pos, cam_roll, cam_pitch, cam_yaw, ...
        fov_x, fov_y, fov_range_min, fov_range_max);
    if sum(A*face_center > b) > 0      % vialting any one
        in = 0;
        return;
    end
    
end
