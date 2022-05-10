function [mu, extraout] = ctrlSisoLinearFeedback(obj, t, x, varargin)
    if isempty(obj.K_siso)
        error("obj.K_siso has to be specified.");
    end
    xi = obj.xi_sym(x);
    mu = -obj.K_siso * xi;
    extraout = [];
end