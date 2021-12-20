function [ vertices_struct ] = add_vertex( vertices_struct, vertex, time)
%CALCULATE_HESSIAN Summary of this function goes here
%   Detailed explanation goes here

vertices_struct.vertices(vertices_struct.num_elements+1) = vertex;
vertices_struct.times(vertices_struct.num_elements+1) = time;
vertices_struct.num_fixed = vertices_struct.num_fixed + vertex.num_fixed;
vertices_struct.num_elements = vertices_struct.num_elements + 1;
end