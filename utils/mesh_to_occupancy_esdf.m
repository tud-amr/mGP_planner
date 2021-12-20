function [occupancy, esdf] = mesh_to_occupancy_esdf(FV, dim_x_env, dim_y_env, dim_z_env, ...
    resolution)

    disp('Computing occupancy map and esdf. It may take a while...');

    dim_x = diff(dim_x_env)/resolution;
    dim_y = diff(dim_y_env)/resolution;
    dim_z = diff(dim_z_env)/resolution;
    
    occupancy = false(dim_x, dim_y, dim_z);
    esdf = zeros(dim_x, dim_y, dim_z);
    
    % parfor
    parfor i = 1 : dim_x
        x = dim_x_env(1) + (i-0.5)*resolution;
        for j = 1 : dim_y
            y = dim_y_env(1) + (j-0.5)*resolution;
            for k = 1 : dim_z
                z = dim_z_env(1) + (k-0.5)*resolution;
                pos = [x, y, z];
                dis = point2trimesh(FV, 'QueryPoints', pos);
                esdf(i,j,k) = dis;
                if dis <=0 
                    occupancy(i,j,k) = true;
                end
            end
        end
    end
    
    disp('Finished!');
    
end
