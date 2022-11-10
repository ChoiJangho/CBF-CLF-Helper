classdef Pendulum < CtrlAffineSys
    properties(Constant)
        % theta_lim defines the safety constraint: theta \in [-theta_lim, theta_lim]
        theta_lim_left = -3 * pi /2
        theta_lim_right = 0
        % theta_inv sets the control bound. for theta \in [theta_inv, pi -
        % theta_inv], the control bound cannot hold the pendulum invariant.
        theta_inv = pi / 6
        dtheta_lim = 1.0        
    end
    properties
        m
        l
        gf
    end
    
    methods
        function obj = Pendulum(params)
            if nargin < 1
                params = [];
            end
            if ~isfield(params, 'gf')
                params.gf = 10;
            end       
            if ~isfield(params, 'm')
                params.m = 1;
            end
            if ~isfield(params, 'l')
                params.l = 2;
            end
            params.u_max = params.m * params.l * params.gf * cos(Pendulum.theta_inv);
            params.u_min = -params.u_max;
            obj@CtrlAffineSys(params, 'symbolic', false);
            obj.l = params.l;
            obj.m = params.m;
            obj.gf = params.gf;
        end
        
        function [x, f, g] = defineSystem(obj, params)
            syms theta dtheta real
            x = [theta; dtheta];
            f = [dtheta; - (params.gf / params.l) * sin(theta)];
            g = [0; 1/(params.m * params.l^2)];            
        end
    end
end
