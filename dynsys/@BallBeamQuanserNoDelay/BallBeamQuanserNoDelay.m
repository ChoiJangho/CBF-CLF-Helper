classdef BallBeamQuanserNoDelay < CtrlAffineSysFL
    properties
        gravity = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        theta_saturation = 56 * pi / 180;
    end
    
    methods
        function obj = BallBeamQuanserNoDelay(feedback_gain, V_max)
            if nargin < 2
                V_max = 10;
            end
            params.K_siso = feedback_gain;
            params.rel_deg_y = 3;
            params.u_max = V_max;
            params.u_min = -V_max;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
        end
        function [x, f, g] = defineSystem(obj, params)
            syms p_ball v_ball theta
            a = 5 * obj.gravity * obj.r_arm / (7 * obj.L);
            x = [p_ball; v_ball; theta;];

            % dynamics
            f = [v_ball;
                a * sin(theta);
                0];
            g = [0;0;1];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            y = x(1);
            z = [];
        end
    end
end
