% check if a point is within the triangle in 3D given its local coordinate
function in = inTriangle(q)

    if (q(1) >= 0 && q(1) <= 1 && q(2) >= 0 && q(2) <= 1 && sum(q) <= 1)
        in = true;
    else
        in = false;
    end

end 