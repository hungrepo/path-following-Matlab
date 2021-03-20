function alg_out= alg_convert(alg_new, alg_old, alg_out_old)
    alg_e = alg_new - alg_old;
    if (alg_e > 3 * pi / 2)
        alg_out = alg_out_old - 2 * pi + alg_e;
    elseif (alg_e < -3 * pi / 2)
        alg_out = alg_out_old + 2 * pi + alg_e;
    else
        alg_out = alg_out_old + alg_e;
    end
end
