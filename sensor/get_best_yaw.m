function yaw_best = get_best_yaw(pos, map_parameters)
% Determine the best yaw for taking measurement
    
    switch map_parameters.model_name
        case 'cylinder'
            center_pos = map_parameters.center_pos;
            dx = center_pos(1) - pos(1);
            dy = center_pos(2) - pos(2);
            yaw_best = atan2(dy, dx);
        case 'boeing747'
            x = pos(1);
            y = pos(2);    
            if (abs(y) <= 8)
                center_pos = [x, 0, 0];
                dx = center_pos(1) - pos(1);
                dy = center_pos(2) - pos(2);
                yaw_best = atan2(dy, dx);
            elseif (2*x-y-68 >= 0 && y > 0)
                yaw_best = -pi;
            elseif (2*x+y-68 >= 0 && y < 0)
                yaw_best = -pi;
            else
                yaw_best = 0;
            end
        case 'ucylinder'
            center_pos = map_parameters.center_pos;
            if pos(1) > 9
                center_pos = map_parameters.center_pos_2;
            end
            dx = center_pos(1) - pos(1);
            dy = center_pos(2) - pos(2);
            yaw_best = atan2(dy, dx);
        otherwise
            error('Model name is not determined!');
    end
   
end
