function [A, b] = get_camera_fov_constraints(cam_pos, cam_roll, cam_pitch, cam_yaw, ...
    fov_x, fov_y, fov_range_min, fov_range_max)
% Determine the camera's FOV constraints represented by a set of linear
% inequality constraints
%
% Input:
%   cam_pos: camera position, [3x1], m
%   cam_roll: camera roll, rad
%   cam_pitch: camera pitch, rad
%   cam_yaw: camera yaw, rad
%   fov_x: FOV horizontal angle, rad
%   fov_y: FOV vertical angle, rad
%   fov_range_min: FOV depth range min
%   fov_range_max: FOV depth range max
% ---
% Output:
%   A, b: linear ineuqality constraints, A*p <= b
%   A: [4x3], b: [4x1]


    % compute the four max range "bottom" points under zero roll pitch and yaw
    % and as camera at origin
    bottom_dy_max = fov_range_max * tan(0.5*fov_x);
    bottom_dz_max = fov_range_max * tan(0.5*fov_y);
    upper_left_max = [fov_range_max; bottom_dy_max; bottom_dz_max];
    upper_right_max = [fov_range_max; -bottom_dy_max; bottom_dz_max];
    lower_left_max = [fov_range_max; bottom_dy_max; -bottom_dz_max];
    lower_right_max = [fov_range_max; -bottom_dy_max; -bottom_dz_max];
    
%     % the min fov range
%     bottom_dy_min = fov_range_min * tan(0.5*fov_x);
%     bottom_dz_min = fov_range_min * tan(0.5*fov_y);
%     upper_left_min = [fov_range_min; bottom_dy_min; bottom_dz_min];
%     upper_right_min = [fov_range_min; -bottom_dy_min; bottom_dz_min];
%     lower_left_min = [fov_range_min; bottom_dy_min; -bottom_dz_min];
%     lower_right_min = [fov_range_min; -bottom_dy_min; -bottom_dz_min];
    
    % rotation and translation
    R = rotz(rad2deg(cam_yaw))*rotx(rad2deg(cam_roll))*roty(rad2deg(cam_pitch));
    upper_left_max = R*upper_left_max + cam_pos;
    upper_right_max = R*upper_right_max + cam_pos;
    lower_left_max = R*lower_left_max + cam_pos;
    lower_right_max = R*lower_right_max + cam_pos;
    bottom_center_max = (upper_left_max + upper_right_max + lower_left_max + ...
        lower_right_max) / 4;
%     upper_left_min = R*upper_left_min + cam_pos;
%     upper_right_min = R*upper_right_min + cam_pos;
%     lower_left_min = R*lower_left_min + cam_pos;
%     lower_right_min = R*lower_right_min + cam_pos;
    
    % linear inequality constraints
    [at, bt] = plane_inequality_four_points(cam_pos, upper_left_max, upper_right_max, ...
        bottom_center_max);
    [al, bl] = plane_inequality_four_points(cam_pos, upper_left_max, lower_left_max, ...
        bottom_center_max);
    [ad, bd] = plane_inequality_four_points(cam_pos, lower_left_max, lower_right_max, ...
        bottom_center_max);
    [ar, br] = plane_inequality_four_points(cam_pos, lower_right_max, upper_right_max, ...
        bottom_center_max);
   	A = [at'; al'; ad'; ar'];
    b = [bt; bl; bd; br];    
    
end
