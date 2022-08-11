classdef ExampleSysFl2 < CtrlAffineSysFL
    methods
        function obj = ExampleSysFl2(feedback_gain)
            params.rel_deg_y = 2;
            params.K_siso = feedback_gain;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        
        function [x, f, g] = defineSystem(obj, ~)
            syms x1 x2 real
            x = [x1; x2;];
            f = [-x1^3 + x1 + x2; 0];
            g = [0; 1];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, ~, x)
            y = x(1);
            z = [];
        end
        
        function [u, extraout] = ctrlBackstepping(obj, t, x, varargin)
            kwargs = parse_function_args(varargin{:});
            if isfield(kwargs, 'c')
                c = kwargs.c;
            else
                c = 2;
            end
            x1 = x(1); x2 = x(2);            
            u = -c * (x1 + x2) - (-x1^3 + x1 + x2) - x1;
            extraout = [];
        end
    end
end
        