function [ M, D_F ] = assemble_M_and_D_F( vertices_struct )
%ASSEMBLE_M_AND_D_F Summary of this function goes here
%   Detailed explanation goes here

current_pos_fixed = 1;
current_pos_D_F = 1;
current_pos_free = vertices_struct.num_fixed + 1;

num_elements = vertices_struct.num_elements;
total_num_fixed = vertices_struct.num_fixed;
N = vertices_struct.N;

% NOTE: this M matrix is the transpose of what Charles Richter had in his
% paper (easyer to assemble)
M = zeros( (N + 1) * (num_elements - 1),(N + 1) / 2 * num_elements);
D_F = zeros(total_num_fixed,1);

% handle start vertex
vertex = vertices_struct.vertices(1);
for i=1:1:(N+1)/2
  if vertex.isFixed(i)
      M(i,current_pos_fixed) = 1;
      current_pos_fixed = current_pos_fixed + 1;
      D_F(current_pos_D_F) = vertex.constraints(i);
      current_pos_D_F = current_pos_D_F + 1;
  else
      M(i,current_pos_free) = 1;
      current_pos_free = current_pos_free + 1;
  end
end

%update M matrix in between vertices
for i=2:1:num_elements-1
  vertex = vertices_struct.vertices(i);

  last_vertex_pos = (i-2)*(N+1) + (N+1)/2;

  for n=1:1:(N+1)/2
    if vertex.isFixed(n)
        M(n+last_vertex_pos,current_pos_fixed) = 1;
        M(n+last_vertex_pos + (N+1)/2,current_pos_fixed) = 1;
        current_pos_fixed = current_pos_fixed + 1;
        D_F(current_pos_D_F) = vertex.constraints(n);
        current_pos_D_F = current_pos_D_F + 1;
    else
        M(n+last_vertex_pos,current_pos_free) = 1;
        M(n+last_vertex_pos + (N+1)/2,current_pos_free) = 1;
        current_pos_free = current_pos_free + 1;
    end
  end
end

% handle last vertex
vertex = vertices_struct.vertices(num_elements);

last_vertex_pos = (num_elements-2)*(N+1) + (N+1)/2;
for i=1:1:(N+1)/2
  if vertex.isFixed(i)
      M(i+last_vertex_pos,current_pos_fixed) = 1;
      current_pos_fixed = current_pos_fixed + 1;
      D_F(current_pos_D_F) = vertex.constraints(i);
      current_pos_D_F = current_pos_D_F + 1;
  else
      M(i+last_vertex_pos,current_pos_free) = 1;
      current_pos_free = current_pos_free + 1;
  end
end
%imagesc(M)
%colorbar
%pause
end

