clear all
clear 
clc 

load cylinder_ipp_correlation_I_SPD.mat;
data_mGP = load('cylinder_kernel_5.mat');

trials = fieldnames(logger);

% methods{1} = 'num';
% methods{2} = 'mGP';
% methods{3} = 'I';
% methods{4} = 'SPD';

logger_tmp = logger;
clear logger

for i = 1 : num_trials
    
    logger.(trials{i}).num = logger_tmp.(trials{i}).num;
    logger.(trials{i}).mGP = data_mGP.logger.(trials{i}).ipp;
    logger.(trials{i}).I = logger_tmp.(trials{i}).I;
    logger.(trials{i}).SPD = logger_tmp.(trials{i}).SPD;
    
end

clear logger_tmp trials data_mGP

root_folder = pwd;
save([root_folder, '/eval/spatial_correlation/', ...
    'cylinder_ipp_correlation', '.mat']); 
