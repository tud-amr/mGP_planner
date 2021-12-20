function flag = if_in_los(p1, p2, map_parameters)
% Determine if p1 and p2 are in LoS under the mesh triangulation TR

% Input:
%   - p1: point 1, [3x1]
%   - p2: point 2, [3x1]
%   - map_parameters: contains environment map info
% Output: 
%   - flag: if in los, 0 or 1

    flag = 1;

    dim_x_env = map_parameters.dim_x_env;
    dim_y_env = map_parameters.dim_y_env;
    dim_z_env = map_parameters.dim_z_env;
    dim_env_lower = [dim_x_env(1); dim_y_env(1); dim_z_env(1)];
    resolution = map_parameters.resolution;
    esdf = map_parameters.esdf;

    % sampling from p1 to p2
    p12_vec = p2 - p1;
    p12_norm = norm(p12_vec);
    p12_vec_unit = p12_vec / p12_norm;
    sample_step = resolution;
    num_sample = ceil(p12_norm/sample_step);
    for i = 1 : num_sample
        p_val = p1 + i*sample_step*p12_vec_unit;
        % validate the distance
        idx = floor((resolution+p_val-dim_env_lower)/resolution);
        try 
            dis = esdf(idx(1),idx(2),idx(3));
            if dis <= 0
                flag = 0;
            end
        catch
            flag = 1;
        end
    end

end
