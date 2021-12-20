N=11;

n_segments = 2;
positions = (rand(n_segments,1)-0.5)*10;


v_max = 10;

times = abs(diff([0; positions]))*2/v_max;

times = times+1

clear vertices_struct;
vertices_struct.num_fixed = 0;
vertices_struct.num_elements = 0;
vertices_struct.N = N;

% add vertices
vertices_struct = add_vertex(vertices_struct,create_vertex_start_end(0, N),0);
for i=1:1:n_segments-1;
  vertices_struct = add_vertex(vertices_struct,create_vertex_position(positions(i), N),times(i));
end
vertices_struct = add_vertex(vertices_struct,create_vertex_start_end(positions(n_segments), N),times(n_segments));
tic
p = solve_problem(vertices_struct);
toc

plot_segments(p,times,N);

hold on

plot(cumsum(times),positions,'x')

