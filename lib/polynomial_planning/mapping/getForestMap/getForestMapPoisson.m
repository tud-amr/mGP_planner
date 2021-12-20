function [ map ] = getForestMapPoisson( num_trees, map_size, tree_size)
%MAKEFORESTT
%% user set parameters
heli_radius = tree_size/4; %min distance from tree that is freespace
tree_radius = tree_size/4; %radius of tree trunks
end_area_size = 1; %clears bottom left and top right corner
%map_size = 5; %size of input map in metres
res = map_size*20; %output res of map

%% Old params
% heli_radius = 0.8; %min distance from tree that is freespace
% tree_radius = 0.3; %radius of tree trunks
% end_area_size = 5; %clears bottom left and top right corner
% map_size = 50; %size of input map in metres
% res = 1000; %output res of map

%% run code

circle_locs = map_size*rand(num_trees,2);
radius = tree_radius + heli_radius;

%create tree free start and end
valid_start = or(circle_locs(:,1) > end_area_size, circle_locs(:,2) > end_area_size);
valid_end = or(circle_locs(:,1) < map_size - end_area_size, circle_locs(:,2) < map_size - end_area_size);
valid = and(valid_start,valid_end);

circle_locs = circle_locs(valid,:);

f = figure('visible','off');

%map trees
for i = 1:size(circle_locs,1)
    corners = [circle_locs(i,1)-radius,...
        circle_locs(i,2)-radius,...
        2*radius,...
        2*radius];
    rectangle('Position',corners,'Curvature',1,'FaceColor','k');
end

%transform output to matrix
set(gcf,'units','pixel');
set(gcf,'position',[0,0,res,res]);
set(gcf,'papersize',[res,res]);

axis equal;
axis([0 map_size 0 map_size])
set(gca,'position',[0 0 1 1],'units','normalized')
axis off;

[map, ~] = frame2im(getframe(gcf));
map = map(:,:,1)>0;

close(f);
end

