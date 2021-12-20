ds = linspace(-5, 5, 200);
epsilon = 2;
cs = zeros(size(ds));

for i = 1:length(ds)
  d = ds(i);
  
  if (d < 0)
    cs(i) = -d + 1/2*epsilon;
  elseif (d < epsilon)
    cs(i) = 1/(2*epsilon) * (d-epsilon)^2;
  else
    cs(i) = 0;
  end
end

plot(ds, cs, ds(2:end), diff(cs), ds(3:end), diff(diff(cs)));
