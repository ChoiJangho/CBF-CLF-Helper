classdef BallBeamSastry < CtrlAffineSys
    properties
        G = 9.81;
%         G = 9.81;
    end
    
    methods
        function obj = BallBeamSastry(params)
            obj@CtrlAffineSys(params, 'symbolic');
        end
        function [x, f, g] = defineSystem(obj, ~)
            syms x1 x2 x3 x4
            x = [x1; x2; x3; x4];

            % dynamics
            f = [x2;
                x1*x4^2 - obj.G * sin(x3);
                x4;
                0];
            g = [0;0;0;1];
        end
        function get_tau_from_u(obj, x, u)
            M = params.M;
            tau = 2 * M * x(1) * x(2) * x(4) + M * obj.G * x(1) * cos(x(3)) + ...
                (M * x(1)^2 + params.J) * u;
        end
        function [value, isterminal, direction] = ball_out_of_range(obj, t, x)
            if x(3) > pi/2 - 0.01 || x(3) < -pi/2 + 0.02
                value = -1;
            else
                value = 1;
            end

            isterminal = 1;
            direction = -1;
        end       
    end
end
