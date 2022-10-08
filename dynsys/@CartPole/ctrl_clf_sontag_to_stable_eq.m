function [u, extraout] = ctrl_clf_sontag_to_stable_eq(obj, t, x, varargin)
kwargs = parse_function_args(varargin{:});

gamma_active = false;
if isfield(kwargs, 'gamma_active')
    gamma_active = kwargs.gamma_active;
end

x_shift = x;
x_shift(2) = clip_angle(x_shift(2) - pi);

V = x_shift' * obj.P_stable_eq * x_shift;
LfV = 2 * x_shift' * obj.P_stable_eq * obj.f(x);
LgV = 2 * x_shift' * obj.P_stable_eq * obj.g(x);
if gamma_active
    b = LfV + obj.clf_rate_stable_eq * V;
else
    b = LfV;
end

if LgV ~= 0
    u_raw = - (b + sqrt(b^2 + LgV^4)) / LgV;
else
    u_raw = 0;
end
u = obj.clipInput(u_raw);
extraout = [];
end