% 1D example

cov_func = {'covMaterniso', 3};
hyp.cov =  [1.3 0.3];

Z = 1:40;
Kss = feval(cov_func{:}, hyp.cov, Z', []);

X = 1:10;
Kss2 = feval(cov_func{:}, hyp.cov, Z', X');