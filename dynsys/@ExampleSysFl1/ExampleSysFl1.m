classdef ExampleSysFl1 < CtrlAffineSysFL
    methods
        function obj = ExampleSysFl1(feedback_gain)
            params.rel_deg_y = 2;
            params.K_siso = feedback_gain;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        
        function [x, f, g] = defineSystem(obj, ~)
            syms x1 x2 x3 real
            x = [x1; x2; x3];
            f = [x3 - x2 ^3;
                -x2;
                x1^2 - x3];
            g = [0; -1; 1];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, ~, x)
            y = x(1);
            z = x(2) + x(3);
        end
    end
end
        