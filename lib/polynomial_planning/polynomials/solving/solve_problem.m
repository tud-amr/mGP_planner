function [ p ] = solve_problem( vertices_struct )
%SOLVE_PROBLEM Summary of this function goes here
%   Detailed explanation goes here

N = vertices_struct.N;
total_num_fixed = vertices_struct.num_fixed;

% NOTE: this M matrix is the transpose of what Charles Richter had in his
% paper
[ M, D_F] = assemble_M_and_D_F(vertices_struct);
[ A_inv, R_unordered] = assemble_A_and_R(vertices_struct.times, N);

% assemble and solve problem
R = M'*R_unordered*M;
[m, n] = size(R);

R_FF = R(1:total_num_fixed,1:total_num_fixed);
R_FP = R(1:total_num_fixed,total_num_fixed+1:n);
R_PF = R(total_num_fixed+1:n,1:total_num_fixed);

R_PP = R(total_num_fixed+1:m,total_num_fixed+1:n);

%imagesc(R_PP)
%figure(1);
%imagesc(R_FP-R_PF)


% [L,U] = lu(R_PP);
% y = L\(R_FP'*D_F);
% D_P = -U\y;
D_P = -R_PP\(R_FP'*D_F);
p = A_inv*M*[D_F;D_P];
end