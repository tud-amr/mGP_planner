function [ A_inv, R_unordered ] = assemble_A_and_R( times, N )
%ASSEMBLE_A_AND_Q Summary of this function goes here
%   Detailed explanation goes here
num_elements = length(times);

R_unordered = zeros((N + 1) * (num_elements - 1));
A_inv = zeros((N + 1) * (num_elements - 1));

for i=2:1:num_elements
  time = times(i);
  
  R_unordered((i-2)*(N+1) + 1:(i-1)*(N+1),...
    (i-2)*(N+1)+1:(i-1)*(N+1)) = compute_R_unordered(time);
  A_inv((i-2)*(N+1) + 1:(i-1)*(N+1),...
    (i-2)*(N+1)+1:(i-1)*(N+1)) = compute_A_inv(time);

end

end

