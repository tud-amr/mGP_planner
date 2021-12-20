function k = cov_materniso_3(d, sigma_f, l)

    k = sigma_f^2 * (1+sqrt(3)*d/l) * exp(-sqrt(3)*d/l);    

end
