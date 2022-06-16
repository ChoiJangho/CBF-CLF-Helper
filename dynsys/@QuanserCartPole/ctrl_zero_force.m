function [u, extraout] = ctrl_zero_force(obj, t, x, varargin)
% Apply the voltage so the resulting force applied to the cart from the
% motor is equivalent to 0.
u = obj.constant_motor_drag * x(3) / obj.constant_V2F;
extraout = [];        
end

