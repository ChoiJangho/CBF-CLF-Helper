function [u, extraout] = ctrlPD(obj, t, x, varargin)
kwargs = parse_function_args(varargin{:});
if isfield(kwargs, 'k_p')
    k_p = kwargs.k_p;
else
    k_p = 4.5;
end
if isfield(kwargs, 'k_v')
    k_v = kwargs.k_v;
else
    k_v = 3;
end

theta_d = - k_p * x(1) - k_v * x(2);
theta_d = min(theta_d, obj.theta_saturation);
theta_d = max(theta_d, -obj.theta_saturation);

k_servo = 12;    
u = k_servo * (theta_d - x(3));
extraout = [];