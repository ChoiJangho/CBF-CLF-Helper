function [u, extraout] = ctrl_lqr_feedback_to_stable_eq(obj, t, x, varargin)
kwargs = parse_function_args(varargin{:});

x_shift = x;
x_shift(2) = clip_angle(x_shift(2) - pi);

u = - obj.K_stable_eq * x_shift;
extraout = [];

end