function [row, col] = world_to_table_pos(pos, map)
  num_rows = map.GridSize(1);
  num_cols = map.GridSize(2);
  
  pos = [pos(2) pos(1)];
  
  pos_grid = (pos - map.GridLocationInWorld) * map.Resolution;

  row = num_rows-int32(pos_grid(1));
  col = int32(pos_grid(2));
    
  if (row > num_rows || row < 1 || col > num_cols || col < 1)
    row = 1;
    col = 1;
  end
end