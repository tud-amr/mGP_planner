function [a, b] = plane_inequality_four_points(p1, p2, p3, p4)
% Determine the plane inequality constraint from four points
% 
% Input:
%   p1: point 1 on the plane, [3x1]
%   p2: point 2 on the plane, [3x1]
%   p3: point 3 on the plane, [3x1]
%   p4: point 4 out the plane satisfying inequality constraint, [3x1]
% ---
% Output:
%   a, b: plane equation parameters, a'*p <= b
%   A: [3x1], b: [1]

    % plane parameters
    p12 = p1 - p2;
    p13 = p1 - p3;
    plane_normal = cross(p12, p13);
    a = plane_normal / norm(plane_normal);
    b = a'*p1;

    % determine direction
    if a'*p4 - b >= 0 
        a = -a;
        b = -b;
    end
    
end
