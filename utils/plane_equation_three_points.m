function [a, b] = plane_equation_three_points(p1, p2, p3)
% Determine the plane equation from three points
% 
% Input:
%   p1: point 1, [3x1]
%   p2: point 2, [3x1]
%   p3: point 3, [3x1]
% ---
% Output:
%   a, b: plane equation parameters, a'*p = b
%   A: [3x1], b: [1]


    p12 = p1 - p2;
    p13 = p1 - p3;
    plane_normal = cross(p12, p13);
    a = plane_normal / norm(plane_normal);
    b = a'*p1;

end
