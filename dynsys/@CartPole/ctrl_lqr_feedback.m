function [u, extraout] = ctrl_lqr_feedback(obj, t, x, varargin)
    kwargs = parse_function_args(varargin{:});

    if ~isfield(kwargs, 'active_input_bound')
        active_input_bound = true;
    else
        active_input_bound = kwargs.active_input_bound;
    end
    u = - obj.K_origin * x;
extraout = [];
end