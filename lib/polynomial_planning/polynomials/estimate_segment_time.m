% std::vector<double> estimateSegmentTimes(const Vertex::Vector& vertices, double v_max, double a_max,
%                                          double magic_fabian_constant) {
%   std::vector<double> segment_times;
%   segment_times.reserve(vertices.size() - 1);
%   for (size_t i = 0; i < vertices.size() - 1; ++i) {
%     Eigen::VectorXd start, end;
%     vertices[i].getConstraint(derivative_order::POSITION, &start);
%     vertices[i + 1].getConstraint(derivative_order::POSITION, &end);
%     double distance = (end - start).norm();
%     double t = distance / v_max * 2 * (1.0 + magic_fabian_constant * v_max / a_max * exp(-distance / v_max * 2));
%     segment_times.push_back(t);
%   }
%   return segment_times;
% }


function t = estimate_segment_time(pos1, pos2, v_max, a_max, constant) 
if (nargin < 5)
  % Magic nfabian constant. :(
  constant = 6.5;
end

distance = norm(pos2 - pos1);
t = 2*distance/v_max*(1+constant*v_max/a_max * exp(-distance/v_max*2));

if (t < 0.01)
  t = 0.01;
end
end