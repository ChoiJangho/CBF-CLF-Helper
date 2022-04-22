classdef BallBeamSastryApprox1 < CtrlAffineSysFL
    properties
        G = 9.81;
    end
    
    methods
        function obj = BallBeamSastryApprox1(params)
            params.rel_deg_y = 4;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        function [x, f, g] = defineSystem(obj, ~)
            syms x1 x2 x3 x4
            x = [x1; x2; x3; x4];

            % dynamics
            f = [x2;
                - obj.G * sin(x3);
                x4;
                0];
            g = [0;0;0;1];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            y = x(1);
            z = [];
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