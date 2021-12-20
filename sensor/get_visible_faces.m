function [F_visible, faces_visible] = get_visible_faces(num_faces, F_points, F_center, ...
    F_normal, cam_pos, cam_roll, cam_pitch, cam_yaw, sensor_parameters)

    F_visible = zeros(num_faces, 1);
    faces_visible = [];
    for iFace = 1 : num_faces
        in = if_in_cam_fov(F_points(iFace, :, :), F_center(iFace,:)', F_normal(iFace,:)', ...
            cam_pos, cam_roll, cam_pitch, cam_yaw, sensor_parameters);
        F_visible(iFace) = in;
        if in
            faces_visible = [faces_visible; iFace];
        end
    end

end
