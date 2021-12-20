syms t real;

N = 11;

optimize_for_derivative = 2;

r=optimize_for_derivative;

syms Q;
for i=1:1:N+1
    for l=1:1:N+1
        Q(i, l) = 0;
    end
end

for i=r:1:N
    for l=r:1:N
        coeff = prod((i - (0:r - 1)) .* (l - (0:r - 1)));
        Q(i+1, l+1) = 2 * coeff * ...
            t^(i + l - 2 * r + 1) / (i + l - 2 * r + 1);
    end
end


syms A0;%
for r = 1:1:(N+1)/2
    for n = 1:1:N+1
        A0(r,n) = 0;
    end
end

syms AT;%
for r = 1:1:(N+1)/2
    for n = 1:1:N+1
        AT(r,n) = 0;
    end
end


A0(1,1) = 1;
for r = 1:1:(N-1)/2
    A0(r+1,r+1) = factorial(r);
end

AT(1,1) = 1;
for n = 0:1:N
    AT(1,n + 1) = t^(n);
end
for r = 1:1:(N-1)/2
    for n = r:1:N
        AT(r+1,n+1) = prod(n - (0:r - 1)) * t^(n-r);
    end
end


AA = [A0;AT];

A_inv = inv(AA);

R_unordered = A_inv' * Q * A_inv;

disp('generating code to compute R_unordered ----' )
matlabFunction(R_unordered, 'file', 'compute_R_unordered', 'vars', {t});

disp('generating code to compute A_inv ----' )
matlabFunction(A_inv, 'file', 'compute_A_inv', 'vars', {t});

