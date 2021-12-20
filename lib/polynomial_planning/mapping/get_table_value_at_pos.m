function [value] = get_table_value_at_pos(pos, map, table, out_of_bounds_value)
  if (nargin < 4)
    out_of_bounds_value = 1;
  end

  num_rows = map.GridSize(1);
  num_cols = map.GridSize(2);
  
  pos = [pos(2) pos(1)];
  
  pos_grid = (pos - map.GridLocationInWorld) .* map.Resolution;
  % Flip row value.
  pos_grid(1) = num_rows - pos_grid(1);
  
  row = int32(round(pos_grid(1)));
  col = int32(round(pos_grid(2)));
    
  % Anything on the border is out of bounds.
  if (row > num_rows-1 || row < 2 || col > num_cols-1 || col < 2)
    value = out_of_bounds_value;
    return;
  end
  
  % Now actually interpolate (linearly) between neighboring positions.
  % Check row: go above or below
  value_at_point = table(row, col);
  row_diff = pos_grid(1) - double(row);
  col_diff = pos_grid(2) - double(col);
  comp_val_row = table(row+sign(row_diff), col);
  comp_val_col = table(row, col+sign(col_diff));
  comp_diag = table(row+sign(row_diff), col+sign(col_diff));
  
  row_diff = abs(row_diff);
  col_diff = abs(col_diff);
  
  %  val_row = (1 - row_diff - col_diff) * value_at_point + (row_diff * comp_val_row) + (col_diff * comp_val_col);

  %val_row = (1 - row_diff) * value_at_point + (row_diff * comp_val_row);
  %val_col = (1 - col_diff) * value_at_point + (col_diff * comp_val_col);
  
  value = (1 - row_diff)*(1 - col_diff)*value_at_point + ...
    (row_diff)*(1-col_diff)*comp_val_row + ...
    (1-row_diff)*(col_diff)*comp_val_col + ...
    (row_diff)*(col_diff)*comp_diag;
end

