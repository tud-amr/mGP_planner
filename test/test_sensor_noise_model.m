clear all
clear 
clc 

sensor_parameters.sensor_coeff_A = 0.05;
sensor_parameters.sensor_coeff_B = 0.2;

incidence = 1;          % cosine value of the face incidence angle
range_sensor = 0 : 0.1 : 30;
Num = length(range_sensor);
var_sensor = zeros(1, Num);
for i = 1 : Num
    var_sensor(i) = sensor_model_inspect(range_sensor(i), incidence, sensor_parameters);
end

figure;
hold on; grid on; box on;
xlabel('range [m]');
ylabel('variance');
plot(range_sensor, var_sensor, 'Color', [235,114,70]./256, 'LineWidth', 2.5);
