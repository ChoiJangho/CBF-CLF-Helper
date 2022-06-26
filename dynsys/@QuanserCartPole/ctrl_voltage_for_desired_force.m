function [u, extraout] = ctrl_voltage_for_desired_force(obj, t, x, f_desired, varargin)
% Apply the voltage so the resulting force applied to the cart from the
% motor is equivalent to the desired force
% f_desired can be either a numeric or a function handle.
% This can be considered as a low-level controller of converting the force
% control command to the motorl voltage command.
if isa(f_desired, 'function_handle')
    [f_desired_, extraout] = f_desired(t, x, varargin{:});
elseif isa(f_desired, 'numeric')
    f_desired_ = u_ref;
    extraout = [];
end
u_raw = (f_desired_ + obj.constant_motor_drag * x(3)) / obj.constant_V2F;
extraout.f_desired = f_desired_;
u = obj.clipInput(u_raw);
extraout.u_raw = u_raw;
end