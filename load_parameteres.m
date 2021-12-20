function [map_parameters, sensor_parameters, planning_parameters, ...
    optimization_parameters, matlab_parameters] = ...
        load_parameteres(model)

    disp('Parameters loading...');
    
    %% sensor parameter, fixed
    sensor_parameters.cam_roll = 0;
    sensor_parameters.cam_pitch = deg2rad(15);
    sensor_parameters.cam_yaw = 0;
    sensor_parameters.fov_x = deg2rad(60);
    sensor_parameters.fov_y = deg2rad(60);
    sensor_parameters.fov_range_min = 2;
    sensor_parameters.fov_range_max = 8;
    sensor_parameters.incidence_range_min = cos(deg2rad(70));
    sensor_parameters.sensor_coeff_A = 0.05;
    sensor_parameters.sensor_coeff_B = 0.2;
    
    %% map parameters
    % dimension
    map_parameters.model_name = model.name;
    if strcmp(model.name, 'cylinder')
        map_parameters.dim_x_env = [-14, 14];
        map_parameters.dim_y_env = [-14, 14];
        map_parameters.dim_z_env = [2, 30];
        map_parameters.center_pos = [0; 0; 11];
    elseif strcmp(model.name, 'boeing747')
        map_parameters.dim_x_env = [-8 80];
        map_parameters.dim_y_env = [-40 40];
        map_parameters.dim_z_env = [-4 16];
        map_parameters.center_pos = [36; 0; 0];
    elseif strcmp(model.name, 'ucylinder')
        map_parameters.dim_x_env = [-8 26];
        map_parameters.dim_y_env = [-8 16];
        map_parameters.dim_z_env = [2 28];
        map_parameters.center_pos = [4; 4; 11];
        map_parameters.center_pos_2 = [14; 4; 11];
    else
        error('Model name not determined!');
    end
    % mesh triangulation
    TR = model.TR;
    map_parameters.TR = TR;
    map_parameters.num_faces = size(TR.ConnectivityList, 1);
    map_parameters.valid_faces = model.valid_faces;
    map_parameters.num_valid_faces = length(model.valid_faces);
    map_parameters.F_normal = faceNormal(TR);
    map_parameters.F_center = incenter(TR);
    map_parameters.F_points = zeros(map_parameters.num_faces, 3, 3);
    for iFace = 1 : map_parameters.num_faces
        map_parameters.F_points(iFace, :, 1) = TR.Points(TR.ConnectivityList(iFace, 1), :);    % 1x3
        map_parameters.F_points(iFace, :, 2) = TR.Points(TR.ConnectivityList(iFace, 2), :);
        map_parameters.F_points(iFace, :, 3) = TR.Points(TR.ConnectivityList(iFace, 3), :);
    end
    % transform mesh to voxel
    map_parameters.resolution = 0.5;
    map_parameters.occupancy = model.occupancy;    
    % compute the esdf
    map_parameters.esdf = model.esdf; 
    % load the temperature field
    map_parameters.temperature_field = model.temperature_field;
    % max range for lattice search
    map_parameters.lattice_range = 16;  % 8, 16
    % kernel choice
    map_parameters.kernel_choice = 5;   % 0-I; 1-Random SPD; 2-Matern; 3-SE; 4-Heat; 5-Geo Matern
    % matern kenel function parameters
    map_parameters.sigma_f = exp(0.3);  % 0.01, 0.3, 0.6
    map_parameters.l = exp(1.3); % 0.2, 1.3, 2.0
    % heat kernel parameters
    map_parameters.sigma_h = 4.0;
    map_parameters.diff_f = 24;
    
    %% trajectory planning parameters
    planning_parameters.safe_radius = 0.6;      % safe radius, [m]
    planning_parameters.max_vel = 4;            % [m/s]
    planning_parameters.max_acc = 3;            % [m/s^2]
    planning_parameters.plan_yaw = 0;           % if also plan yaw polynomial trajectory
    planning_parameters.max_yaw_rate = deg2rad(90); % [rad/s]
    planning_parameters.time_budget = 120;      % 120, 240
    planning_parameters.lambda = 0.001;         % parameter to control 
                                                % exploration-exploitation 
                                                % trade-off in objective
    planning_parameters.measurement_frequency = 0.2;
    planning_parameters.use_threshold = 1;
    planning_parameters.lower_threshold = 0.0;
    planning_parameters.obj = 'rate';    % 'rate'/'exponential'
    planning_parameters.control_points = 4;     % 5, 4
    
    %% global optimization paramters
    optimization_parameters.opt_method = 'cmaes'; % 'aco'
    optimization_parameters.max_iters = 45;
    optimization_parameters.opt_yaw = 0;
    optimization_parameters.cov_x = 5;
    optimization_parameters.cov_y = 5;
    optimization_parameters.cov_z = 5;
    optimization_parameters.cov_yaw = 3;
    
    %% matlab parameters
    matlab_parameters.visualize_map = 1;
    matlab_parameters.visualize_path = 1;
    matlab_parameters.visualize_cam = 0;

    disp('Parameters loaded!');
    
end