classdef Car4D < CtrlAffineSys
    methods
        function [s, f, g] = defineSystem(obj, params)
            % theta: heading
            syms x y theta v
            
            s = [x; y; theta; v];
            f = [v*cos(theta);v*sin(theta);0;0];
            % u1: yaw rate, u2: longitudinal acceleration.
            g = [0, 0; 0, 0; 1, 0; 0, 1;];
        end
        
        function cbf = defineCbf(obj, params, x_sym)
            xo = params.xo;
            yo = params.yo;
            Ro = params.Ro;
            p_x = x_sym(1);
            p_y = x_sym(2);
            theta = x_sym(3);
            v = x_sym(4);
%             lx = (p_x - xo)^2 + (p_y - yo)^2 - Ro^2;            
%             dlx = 2 * (p_x - xo) * v * cos(theta) + ...
%                 2 * (p_y -yo) * v * sin(theta);
%             cbf = dlx + params.gamma_l * lx;
            
%            smooth_margin = (1 - sqrt(1 - (1 - 2 * v / params.v_max)^2));
            smooth_margin = 0;
            stopping_distance = 0.5 * v^2 / params.max_acc;
            avoid_center = [-0.5 * stopping_distance * cos(theta) + xo;
                -0.5 * stopping_distance * sin(theta) + yo];
            avoid_radius = Ro + stopping_distance * 0.5 + smooth_margin;
            cbf = sqrt((p_x - avoid_center(1))^2 + (p_y - avoid_center(2))^2) - avoid_radius;
        end
    end 
end