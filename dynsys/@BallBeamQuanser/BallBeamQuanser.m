classdef BallBeamQuanser < CtrlAffineSysFL
    properties
        gravity = 9.81;
        r_arm = 0.0254;
        L = 0.4255;
        K = 10;
        tau = 0.1;
        theta_saturation = 56 * pi / 180;
        dynsys_no_delay
    end
    
    methods
        function obj = BallBeamQuanser(feedback_gain, V_max)
            if nargin < 2
                V_max = 10;
            end
            params.K_siso = feedback_gain;
            params.rel_deg_y = 4;
            params.u_max = V_max;
            params.u_min = -V_max;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
            eps = 5;
            s0 = 1.1;
            obj.dynsys_no_delay = BallBeamQuanserNoDelay([s0^3, 3*s0^2/eps, 3*s0/eps^2]);
        end
        function [x, f, g] = defineSystem(obj, params)
            syms p_ball v_ball theta dtheta
%             p_ball = x(1);
%             v_ball = x(2);
%             theta = x(3);
%             dtheta = x(4);
            a = 5 * obj.gravity * obj.r_arm / (7 * obj.L);
            x = [p_ball; v_ball; theta; dtheta];

            % dynamics
            f = [v_ball;
                a * sin(theta);
                dtheta;
                -dtheta/obj.tau];
            g = [0;0;0;obj.K/obj.tau];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            y = x(1);
            z = [];
        end
    end
end
