function map = create_random_map(width_m, height_m, resolution, num_samples, inflation)
%CREATE_MAP Create a default map for testing.
%width_m = 10;
%height_m = 10;
% Resolution is inverse of what we think of as resolution. ;)
if (nargin < 3)
  width_m = 4;
  height_m = 4;
  resolution = 10;
  num_samples = 50;
  inflation = 0.1;
end

map = robotics.BinaryOccupancyGrid(width_m, height_m, resolution);

% Density should be a percentage. Probably. I dunno.
% Maybe should draw this number from poisson distribution.
%num_samples = 100;

r = rand(num_samples, 2);
% Scale back to full range.
r(:, 1) = r(:, 1) * width_m;
r(:, 2) = r(:, 2) * height_m;

setOccupancy(map, r, 1);

inflate(map, inflation);

%show(map)

% Convert to grayscale and then black and white image based on arbitrary
% threshold.
%grayimage = rgb2gray(image);
%bwimage = grayimage < 0.5;

% Use black and white image as matrix input for binary occupancy grid
%map = robotics.BinaryOccupancyGrid(bwimage, resolution_m);

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

