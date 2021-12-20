function h = plot_trajectory_cline(time, pos, yaw)

    % Visualize a trajectory
    
    % Input:
    %   - ax: axis plot handle
    %   - time: time step, [Nx1]
    %   - pos: position, [NxD]
    % Output:
    %   - h: plot handle
    
    h = cline(pos(:,1), pos(:,2), pos(:,3), time);
    set(h, 'LineWidth', 2.5);
    
    N = size(pos, 1);
    for i = 1 : N
        u = 2*cos(yaw(i));
        v = 2*sin(yaw(i));
        w = 0;
        quiver3(pos(i,1), pos(i,2), pos(i,3), u, v, w, ...
            'Color', [255,153,153]/256, 'LineWidth', 1.0, 'MaxHeadSize', 0.6);
    end
    
    
end
