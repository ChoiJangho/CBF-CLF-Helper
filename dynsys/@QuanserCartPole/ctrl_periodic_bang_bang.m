function [u, extraout] = ctrl_periodic_bang_bang(obj, t, x, T, varargin)
if nargin < 4
    T = 1;
end
    if t - floor(t / T)*T < 0.5 * T
        u = obj.u_max;
    else
        u = obj.u_min;
    end
extraout = [];    
    
end

