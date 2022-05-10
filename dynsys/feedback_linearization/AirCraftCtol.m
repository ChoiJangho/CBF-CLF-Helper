classdef AirCraftCtol < CtrlAffineSysFL
    properties
        epsilon; % determines the correlation between generating pitching moment and downforce.
        aD = 2; % lift coeff.
        aL = 30; % drag coeff.
        c = 6;
    end
    
    methods
        function obj = AirCraftCtol(params)
            if ~isfield(params, 'epsilon')
                params.epsilon = 0.1;
            end
%             if params.epsilon == 0            
%                 params.rel_deg_y = [2; 3];
%             else
%                 params.rel_deg_y = [2; 2];
%             end
            obj@CtrlAffineSysFL(params, 'symbolic', 'mimo');
        end
        function [x, f, g] = defineSystem(obj, params)
            syms p dx z dz theta dtheta real;
            x = [p; dx; z; dz; theta; dtheta];
            gamma = atan2(dz, dx);
            alpha = theta - gamma;
            Rtheta = [cos(theta), -sin(theta); sin(theta), cos(theta)];
            Ralpha = [cos(alpha), -sin(alpha); sin(alpha) cos(alpha)];
            L = obj.aL * (dx^2 + dz^2) * (1 + obj.c * alpha);
            D = obj.aD * (dx^2 + dz^2);
            f_ddxz = Rtheta * (Ralpha' * [-D ; L]) + [0; -1];
            g_ddxz = Rtheta * [1, 0; 0, -params.epsilon];
            f = [dx; f_ddxz(1); dz; f_ddxz(2); dtheta; 0];
            g = [0, 0;
                g_ddxz(1, :);
                0, 0;
                g_ddxz(2, :);
                0, 0;
                0, 1];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            y = [x(1); x(3)];
            if params.rel_deg_y(2) == 2
                z = [x(5); x(6)];
            else
                z = x(5);
            end
        end
    end
end
