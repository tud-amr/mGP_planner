% plotting data
close all
clear all
clear 
clc 

load cylinder_ipp_correlation.mat

rescale_factor = 1;
%rescale_factor = 0.75;
text_size = 20;
line_width = 2.5;

do_plot = 1;
do_print = 1;
show_legend = 1;

paper_pos = [0, 0, 6, 4];

trials = fieldnames(logger);
methods = fieldnames(logger.trial1);

time_vector = 0:5:120;

P_traces = zeros(length(methods)-1,length(time_vector));
rmses = zeros(length(methods)-1,length(time_vector));
wrmses = zeros(length(methods)-1,length(time_vector));
mlls = zeros(length(methods)-1,length(time_vector));
wmlls = zeros(length(methods)-1,length(time_vector));

for i = 1:length(trials)
    
    for j = 2:length(methods)
       
        try
           time = logger.(trials{i}).(methods{j}).times;
        catch
           disp(['Cant find ', trials{i}, ' ' methods{j}])
           break;
        end
            
        P_trace = logger.(trials{i}).(methods{j}).P_traces;
        rmse = logger.(trials{i}).(methods{j}).rmses;
        wrmse = logger.(trials{i}).(methods{j}).rmses;
        mll = logger.(trials{i}).(methods{j}).mlls;
        wmll = logger.(trials{i}).(methods{j}).wmlls;

        ts = timeseries(P_trace, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        P_traces(j-1,:,i) = ts_resampled.data';
        
        ts = timeseries(rmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        rmses(j-1,:,i) = ts_resampled.data';
     
        ts = timeseries(wrmse, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        wrmses(j-1,:,i) = ts_resampled.data';
 
        ts = timeseries(mll, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        mlls(j-1,:,i) = ts_resampled.data';

        ts = timeseries(wmll, time);
        ts_resampled = resample(ts, time_vector, 'zoh');
        wmlls(j-1,:,i) = ts_resampled.data';

    end
    
end

% Find means and medians.
mean_P_traces = sum(P_traces,3)./length(trials);
mean_rmses = sum(rmses,3)./length(trials);
mean_wrmses = sum(wrmses,3)./length(trials);
mean_mlls = sum(mlls,3)./length(trials);
mean_wmlls = sum(wmlls,3)./length(trials);
median_P_traces = median(P_traces,3);
median_rmses = median(rmses,3);
median_wrmses = median(wrmses,3);
median_mlls = median(mlls,3);
median_wmlls = median(wmlls,3);

% Find confidence intervals
% http://ch.mathworks.com/matlabcentral/answers/159417-how-to-calculate-the-confidence-interva
SEM_P_traces = [];
SEM_rmses = [];
SEM_wrmses = [];
SEM_mlls = [];
SEM_wmlls = [];

for j = 2:length(methods)

    SEM_P_traces(j-1,:) = std(squeeze(P_traces(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials));
    SEM_rmses(j-1,:) = (std(squeeze(rmses(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_wrmses(j-1,:) = (std(squeeze(wrmses(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_mlls(j-1,:) = (std(squeeze(mlls(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    SEM_wmlls(j-1,:) = (std(squeeze(wmlls(j-1,:,:))', 'omitnan')/...
        sqrt(length(trials)));
    
end

% Symmetric
ts = tinv(0.95, length(trials)-1);

colours = [0    0.4470    0.7410;
    0.8500    0.3250    0.0980;
    0.9290    0.6940    0.1250;
    0.4940    0.1840    0.5560;
    0.4660    0.6740    0.1880];
    %0.6350    0.0780    0.1840;
    %0.3010    0.7450    0.9330;
    %0.1379    0.1379    0.0345];
 transparency = 0.3;
 

%% PLOTTING %%

if (do_plot)
        
    fig_P = figure;
    %% Trace of P %%
    hold on
    h = zeros(length(methods)-1,1);
    boundedline(time_vector, mean_P_traces(1,:), SEM_P_traces(1,:)*ts, ...
        time_vector, mean_P_traces(2,:), SEM_P_traces(2,:)*ts, ...
        time_vector, mean_P_traces(3,:), SEM_P_traces(3,:)*ts, ... 
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:3
        P_trace = mean_P_traces(i,:);
        h(i) = plot(time_vector, P_trace, 'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('Trace(P)');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'off'      , ...
        'YMinorTick'  , 'off'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YGrid'       , 'on'      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    if (show_legend)
        h_legend = legend(h, 'mGP', ...
            'Identity', 'random SPD', ...
            'FontName', 'HelveticaNarrow');
    end

    axis([0 time_vector(end) 0 4000])
    rescale_axes(rescale_factor);
 %   pbaspect(gca, [1 2 1])
    hold off
    print(fig_P, '-dpdf', [pwd, '/cylinder_correlation_P.pdf']);
    
    %% RMSE %%
    fig_RMSE = figure;
    hold on
    boundedline(time_vector, mean_rmses(1,:), SEM_rmses(1,:)*ts, ...
        time_vector, mean_rmses(2,:), SEM_rmses(2,:)*ts, ...
        time_vector, mean_rmses(3,:), SEM_rmses(3,:)*ts, ... 
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:3
        rmse = mean_rmses(i,:);
        h(i) = plot(time_vector, rmse, 'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('RMSE');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:0.05:0.2, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    if (show_legend)
        h_legend = legend(h, 'mGP', ...
            'Identity', 'random SPD', ...
            'FontName', 'HelveticaNarrow');
    end
    
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.24])
%    pbaspect(gca, [1 2 1])
    hold off
    print(fig_RMSE, '-dpdf', [pwd, '/cylinder_correlation_RMSE.pdf']);
    
    %% WRMSE %%
    fig_WRMSE = figure;
    hold on
    boundedline(time_vector, mean_wrmses(1,:), SEM_wrmses(1,:)*ts, ...
        time_vector, mean_wrmses(2,:), SEM_wrmses(2,:)*ts, ...
        time_vector, mean_wrmses(3,:), SEM_wrmses(3,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:3
        wrmse = mean_wrmses(i,:);
        h(i) = plot(time_vector, wrmse, 'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('WRMSE');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , 0:0.05:0.2, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    if (show_legend)
        h_legend = legend(h, 'mGP', ...
            'Identity', 'random SPD', ...
            'FontName', 'HelveticaNarrow');
    end
    
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) 0 0.24])
 %   pbaspect(gca, [1 2 1])
    hold off
  
    %% MLL %%
    fig_MLL = figure;
    hold on
    boundedline(time_vector, mean_mlls(1,:), SEM_mlls(1,:)*ts, ...
        time_vector, mean_mlls(2,:), SEM_mlls(2,:)*ts, ...
        time_vector, mean_mlls(3,:), SEM_mlls(3,:)*ts, ... 
        'alpha', 'cmap', colours, 'transparency', transparency);
     
    for i = 1:3
        mll = mean_mlls(i,:);
        h(i) = plot(time_vector, mll, 'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('MLL');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , -1.5:0.5:0.5      , ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    if (show_legend)
        h_legend = legend(h, 'mGP', ...
            'Identity', 'random SPD', ...
            'FontName', 'HelveticaNarrow');
    end
    
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) -1.5 1.5])
%    pbaspect(gca, [1 2 1])
    hold off

    %% WMLL %%
    fig_WMLL = figure;
    hold on
    boundedline(time_vector, mean_wmlls(1,:), SEM_wmlls(1,:)*ts, ...
        time_vector, mean_wmlls(2,:), SEM_wmlls(2,:)*ts, ...
        time_vector, mean_wmlls(3,:), SEM_wmlls(3,:)*ts, ...
        'alpha', 'cmap', colours, 'transparency', transparency);
    
    for i = 1:3
        wmll = mean_wmlls(i,:);
        h(i) = plot(time_vector, wmll, 'LineWidth', line_width, 'Color', colours(i,:));
    end
    
    h_xlabel = xlabel('Time (s)');
    h_ylabel = ylabel('WMLL');
    set([h_xlabel, h_ylabel], ...
        'FontName'   , 'Helvetica');
    set(gca, ...
        'Box'         , 'off'     , ...
        'TickDir'     , 'out'     , ...
        'TickLength'  , [.02 .02] , ...
        'XMinorTick'  , 'on'      , ...
        'YMinorTick'  , 'on'      , ...
        'YGrid'       , 'on'      , ...
        'XColor'      , [.3 .3 .3], ...
        'YColor'      , [.3 .3 .3], ...
        'YTick'       , -1.5:0.5:0.5, ...
        'LineWidth'   , 1         , ...
        'FontSize'    , text_size, ...
        'LooseInset', max(get(gca,'TightInset'), 0.02));
    
    if (show_legend)
        h_legend = legend(h, 'mGP', ...
            'Identity', 'random SPD', ...
            'FontName', 'HelveticaNarrow');
    end
    
    rescale_axes(rescale_factor);
    axis([0 time_vector(end) -1.5 1.5])
 %   pbaspect(gca, [1 2 1])
    hold off
  
end

%close all;