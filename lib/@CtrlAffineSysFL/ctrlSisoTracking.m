function [mu, extraout] = ctrlSisoTracking(obj, t, x, reference_trajectory, varargin)
% reference_trajectory: a function handle with input t, output xi_desired.
    if isempty(obj.K_siso)
        error("obj.K_siso has to be specified.");
    end
    
    xi = obj.xi_sym(x);
    xi_d = reference_trajectory(t);
    mu = -obj.K_siso * (xi - xi_d);
    extraout.xi_d = xi_d;
end