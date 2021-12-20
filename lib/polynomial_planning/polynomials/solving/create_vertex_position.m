function [ vertex ] = create_vertex_position( p, N, unconstrained)
%add_vertex_position Summary of this function goes here
%   Detailed explanation goes here

vertex.constraints = zeros((N+1)/2);
vertex.constraints(1) = p;
vertex.isFixed = zeros((N+1)/2);
if (nargin < 3 || unconstrained == 0)
  vertex.isFixed(1) = 1;
  vertex.num_fixed = 1;
else
 vertex.num_fixed = 0;
end
end