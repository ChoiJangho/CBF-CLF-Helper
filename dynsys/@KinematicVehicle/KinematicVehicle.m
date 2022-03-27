classdef KinematicVehicle < CtrlAffineSys
    methods
        function [s, f, g] = defineSystem(obj, params)
            % theta: heading, sigma: yaw rate
            syms x y v theta sig
            
            s = [x; y; v; theta; sig];
            if params.unmodeled_factor.active
                steer_offset = params.unmodeled_factor.steer_offset;
                kv = params.unmodeled_factor.velocity_skid_coeff;
                ka = params.unmodeled_factor.angular_skid_coeff;
                mu = params.unmodeled_factor.friction;
                
                f = [v*cos(theta);v*sin(theta);-mu;v*(sig+steer_offset);0];
                % u1: longitudinal acceleration, u2: angular acceleration.
                g = [0, 0; 0, 0; kv, 0; 0, 0; 0, ka];                
            else
                f = [v*cos(theta);v*sin(theta);0;v*sig;0];
                % u1: longitudinal acceleration, u2: angular acceleration.
                g = [0, 0; 0, 0; 1, 0; 0, 0; 0, 1];                
            end
        end
        
        function clf = defineClf(~, params, symbolic_s)
            x = symbolic_s(1);
            y = symbolic_s(2);
            v = symbolic_s(3) - params.v_target;
            theta = symbolic_s(4);
            sig = symbolic_s(5);

            clf = 0.37 * y^2 + 0.52 * y * theta + 3.11 * theta^2 + 0.98 * y * sig + 2.23 * sig * theta + ...
                4.46 * sig^2 - 0.36 * v * y - 0.29 * v * theta + 0.95 * v * sig + 3.86 * v^2;
        end
        function cbf = defineCbf(obj, params, x_sym)
            xo = params.xo;
            yo = params.yo;
            Ro = params.Ro;
            p_x = x_sym(1);
            p_y = x_sym(2);
            v = x_sym(3);
            theta = x_sym(4);
            lx = (p_x - xo)^2 + (p_y - yo)^2 - Ro^2;
            
            dlx = 2 * (p_x - xo) * v * cos(theta) + ...
                2 * (p_y -yo) * v * sin(theta);
            cbf = dlx + params.gamma_l * lx;
        end
    end 
end