function ground_truth_faces_map = create_ground_truth_map(map_parameters)

    F_value = map_parameters.temperature_field;
    % scale F_values to be between 0 and 1
    min_value = min(F_value);
    max_value = max(F_value);
    F_value = (F_value - min_value) / (max_value - min_value);
    
    ground_truth_faces_map = F_value;

end