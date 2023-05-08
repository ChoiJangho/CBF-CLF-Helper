function coeffs = get_force_coeff(obj, xs, us)
Fsts = obj.get_force(xs, us);
coeffs = abs(Fsts(1, :)./ Fsts(2,:));
end
