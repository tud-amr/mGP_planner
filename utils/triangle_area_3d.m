% compute the area of a triangle
function area = triangle_area_3d(p1, p2, p3)

    p21 = p2 - p1;
    p31 = p3 - p1;
    area = 0.5 * norm(cross(p21, p31));

end
