% collecting data
close all
clear all
clear 
clc 

root_folder = pwd;

% If data already exists, want to append to it for the trials it contains.
append_to_logger = 0;

%% Number of trials to run
if (~append_to_logger)
    num_trials = 1;
else
    load cylinder_ipp_correlation.mat
    trials = fieldnames(logger);
    trials = regexp(trials,'\d*','Match');
    trials = [trials{:}];
    trials_names = [];
    for i = 1:length(trials)
        trials_names = ...
            [trials_names; str2num(cell2mat(trials(i)))];
    end
    num_trials = size(trials_names,1);
end

%% Environment
model_name = 'cylinder';
model.name = model_name;
% mesh
data_mesh = load([model_name, '_mesh.mat']);
model.TR = data_mesh.TR;
model.valid_faces = data_mesh.valid_faces;
TR = data_mesh.TR;
% occupancy
data_occupancy = load([model_name, '_map_occupancy']);
model.occupancy = data_occupancy.occupancy; 
% esdf
data_esdf = load([model_name, '_map_esdf']);
model.esdf = data_esdf.esdf; 
% true temperature field
data_temperature_field = load([model_name, '_temperature_field']);
model.temperature_field = data_temperature_field.F_value;


%% Parameters
[map_parameters, sensor_parameters, planning_parameters, optimization_parameters, ...
    matlab_parameters] = load_parameteres(model);

viewpoint_init = [-7.0711   -7.0711    4.0000    0.7854]; %[10, 0, 4, -pi]


%% Collecting data
use_mGP = 1;
use_I = 1;
use_SPD = 1;

%logger = struct;

for i = 1:num_trials

    if (~append_to_logger)
        t = i;
    else
        t = trials_names(i);
    end
    
    logger.(['trial', num2str(t)]).num = t;
       
    try
        if (use_mGP) 
            rng(t, 'twister');
            map_parameters.kernel_choice = 5;
            [metrics, ~] = mGP_cmaes_full_function(...
                viewpoint_init, ...
                map_parameters, sensor_parameters, planning_parameters, ...
                optimization_parameters, matlab_parameters);
            logger.(['trial', num2str(t)]).('mGP') = metrics;
        end
        
        if (use_I)
            rng(t, 'twister');
            map_parameters.kernel_choice = 0;
            [metrics, ~] = mGP_cmaes_full_function(...
                viewpoint_init, ...
                map_parameters, sensor_parameters, planning_parameters, ...
                optimization_parameters, matlab_parameters);
            logger.(['trial', num2str(t)]).('I') = metrics;
        end
        
        if (use_SPD)
            rng(t, 'twister');
            map_parameters.kernel_choice = 1;
            [metrics, ~] = mGP_cmaes_full_function(...
                viewpoint_init, ...
                map_parameters, sensor_parameters, planning_parameters, ...
                optimization_parameters, matlab_parameters);;
            logger.(['trial', num2str(t)]).('SPD') = metrics;
        end
        
        disp(['Completed Trial ', num2str(t)]);
        
    catch
        
        disp(['Failed Trial ', num2str(t)]);
        
    end
    
    save([root_folder, '/eval/spatial_correlation/', map_parameters.model_name, ...
        '_ipp_correlation', '.mat']); 
    
end
