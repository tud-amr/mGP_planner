function [ map ] = getForestMap( id )
%MAKEFORESTT
%id - int between 1 and 48 for the map to load.

%% user set parameters
heli_radius = 0.4; %min distance from tree that is freespace
heli_height = 2; %height of heli off ground
end_area_size = 5; %clears bottom left and top right corner
map_size = 50; %size of input map in metres (hardcoded in trees.mat)
res = 1000; %output res of map

%% run code

load('trees.mat');
trees = trees(id);

%check if heli will hit the tree trunk or canopy
trunk_hit = trees.height > heli_height;
crown_hit = and(trunk_hit, heli_height > trees.crown_min_height);

circle_radii = trees.trunk_radii;
circle_radii(crown_hit) = trees.crown_radii(crown_hit);
circle_radii = circle_radii(trunk_hit);

circle_locs = trees.locs(trunk_hit,:);

%create tree free start and end
valid_start = or(circle_locs(:,1) > end_area_size, circle_locs(:,2) > end_area_size);
valid_end = or(circle_locs(:,1) < map_size - end_area_size, circle_locs(:,2) < map_size - end_area_size);
valid = and(valid_start,valid_end);

circle_radii = circle_radii(valid);
circle_locs = circle_locs(valid,:);

f = figure('visible','off');

%map trees
for i = 1:size(circle_locs,1)
    corners = [circle_locs(i,1)-(circle_radii(i)+ heli_radius),...
        circle_locs(i,2)-(circle_radii(i)+ heli_radius),...
        2*(circle_radii(i)+ heli_radius),...
        2*(circle_radii(i)+ heli_radius)];
    rectangle('Position',corners,'Curvature',1,'FaceColor','k');
end

%transform output to matrix
set(gcf,'units','pixel');
set(gcf,'position',[0,0,res,res]);
set(gcf,'papersize',[res,res]);

axis equal;
axis([0 50 0 50])
set(gca,'position',[0 0 1 1],'units','normalized')
axis off;

[map, ~] = frame2im(getframe(gcf));
map = map(:,:,1)>0;

close(f);
end

