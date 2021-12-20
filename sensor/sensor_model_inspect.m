function [var] = sensor_model_inspect(range, incidence, sensor_parameters)
% Measurement model of sensor to inspect a surface.
%
% Input:
%   range: range from the camera to the face, [m]
%   incidence: cosine value of the face incidence angle
% ---
% Output:
%   var = variance associated with measurement

var = sensor_parameters.sensor_coeff_A .* ...
    (1 - exp(-sensor_parameters.sensor_coeff_B .* range));


end
