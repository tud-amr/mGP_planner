function [range, incidence_cos] = get_incidence_range(...
    face_points, face_center, face_normal, cam_pos)
% Compute the incidence and range from a camera to a facet when it is 
% in the field-of-view (FOV)
%
% Input:
%   face_points: points of the facet triangle, [3x3], m
%   face_center: center of the facet triangle, [3x1], m
%   face_normal: normal of the facet triangle, [3x1], m
%   cam_pos: camera position, [3x1], m
% ---
% Output:
%   incidence_cos: cosine of the incidence angle
%   range: range from the camera to the face, m


    % range computation
    range = norm(cam_pos - face_center);
    range_normal = (cam_pos - face_center)' * face_normal;
    
    % incidence computation, using facet center approximation
    % assuming the triangle is very small
    incidence_cos = range_normal / range;
    
end
