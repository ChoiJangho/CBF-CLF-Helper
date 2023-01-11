function [mu, extraout] = ctrlSisoTracking(obj, t, x, reference_trajectory, varargin)
    kwargs = parse_function_args(varargin{:});
    if isfield(kwargs, 'is_angle')
        is_angle = kwargs.is_angle;
    else
        is_angle = false;
    end
% reference_trajectory: a function handle with input t, output xi_desired.
    if isempty(obj.K_siso)
        error("obj.K_siso has to be specified.");
    end
    
    xi = obj.xi_sym(x);
    xi_d = reference_trajectory(t);
    if ~is_angle        
        mu = -obj.K_siso * (xi - xi_d);
    else
        xi_diff = xi - xi_d;
        xi_diff(1) = pi * sin((xi(1) - xi_d(1))/2);
        mu = -obj.K_siso * xi_diff;
    end
    extraout.xi_d = xi_d;
end