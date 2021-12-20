function k = cov_se(d, sigma_f, l)

    k = sigma_f^2 * exp(-d^2/(2*l^2));    

end
