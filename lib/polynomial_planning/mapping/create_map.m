function map = create_map()
%CREATE_MAP Create a default map for testing.
width_m = 10;
height_m = 10;
% Resolution is inverse of what we think of as resolution. ;)
resolution_m = 10;

%map = robotics.BinaryOccupancyGrid(width_m, height_m, resolution_m);
image = imread('map_maze.png');

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;

% Use black and white image as matrix input for binary occupancy grid
map = robotics.BinaryOccupancyGrid(bwimage, resolution_m);

%map = robotics.BinaryOccupancyGrid(width_m, height_m, resolution_m);
% x = [1.2; 2.3; 3.4; 4.5; 5.6];
% y = [5.0; 4.0; 3.0; 2.0; 1.0];
% 
% setOccupancy(map, [4.0, 4.0; 0, 10; 1, 10; 2, 10], 1);
% setOccupancy(map, [2.0, 1.0], 0);
% setOccupancy(map, [x y], ones(5,1));
% inflate(map, 1.0);

%show(map)
end

