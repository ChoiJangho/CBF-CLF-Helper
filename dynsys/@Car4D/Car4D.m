classdef Car4D < CtrlAffineSys
    properties
        v_max
        v_min
        state_dependent_input_bound
        max_acc
        max_yaw_rate
    end
    
    methods
        function obj = Car4D(params, varargin)
            obj = obj@CtrlAffineSys(params, 'symbolic', varargin{:});            
            obj.v_max = params.v_max;            
            if isfield(params, 'v_min')
                obj.v_min = params.v_min;
            else
                obj.v_min = 0;
            end
            if isfield(params, 'state_dependent_input_bound')
                obj.state_dependent_input_bound = params.state_dependent_input_bound;
            else
                obj.state_dependent_input_bound = false;
            end
            obj.max_acc = params.u_max(2);
            obj.max_yaw_rate = params.u_max(1);
        end
        
        function [s, f, g] = defineSystem(obj, params)
            % theta: heading
            syms x y theta v
            
            s = [x; y; theta; v];
            f = [v*cos(theta);v*sin(theta);0;0];
            % u1: yaw rate, u2: longitudinal acceleration.
            g = [0, 0; 0, 0; 1, 0; 0, 1;];
        end
        
%         function cbf = defineCbf(obj, params, x_sym)
%             xo = params.xo;
%             yo = params.yo;
%             Ro = params.Ro;
%             p_x = x_sym(1);
%             p_y = x_sym(2);
%             theta = x_sym(3);
%             v = x_sym(4);
%             max_acc = obj.u_max_constant(2);
% %             lx = (p_x - xo)^2 + (p_y - yo)^2 - Ro^2;            
% %             dlx = 2 * (p_x - xo) * v * cos(theta) + ...
% %                 2 * (p_y -yo) * v * sin(theta);
% %             cbf = dlx + params.gamma_l * lx;
%                         
%             if params.apply_cbf_smooth_margin
%                 smooth_factor = 0.25;
%                 smooth_margin = smooth_factor * (1 - sqrt(1 - (1 - 2 * v / params.v_max)^2));
%             else                
%                 smooth_margin = 0;
%             end
%             stopping_distance = 0.5 * v^2 / max_acc;
%             
%             avoid_center = [-0.5 * stopping_distance * cos(theta) + xo;
%                 -0.5 * stopping_distance * sin(theta) + yo];
%             avoid_radius = Ro + stopping_distance * 0.5 + smooth_margin;
%             cbf = sqrt((p_x - avoid_center(1))^2 + (p_y - avoid_center(2))^2) - avoid_radius;
%         end
        
        function cbf = defineCbf(obj, params, x_sym)
            xo = params.xo;
            yo = params.yo;
            Ro = params.Ro;
            p_x = x_sym(1);
            p_y = x_sym(2);
            theta = x_sym(3);
            v = x_sym(4);
            omega = obj.u_max_constant(1);
            max_acc = obj.u_max_constant(2);
            v_min = params.v_min_desired;
            v_max = params.v_max;
            r_steer = v_max / omega;
            steering_distance = Ro * (sqrt(1 + 2*r_steer / (Ro * omega)) -1 );
            d = 0.5 * steering_distance;
            tau = 0.5 * v_max / max_acc; 
            distance_margin = d + tau * (v - v_min);
            
            avoid_center = [-distance_margin * cos(theta) + xo;
                -distance_margin * sin(theta) + yo];
            avoid_radius = Ro + distance_margin;
            cbf = sqrt((p_x - avoid_center(1))^2 + (p_y - avoid_center(2))^2) - avoid_radius;
        end        
        
        
        function u = u_max(obj, t, x)
            v = x(4);
            u = obj.u_max_constant;
            if v>= obj.v_max && obj.state_dependent_input_bound
                u(2) = 0;
            end                
        end
        
        function u = u_min(obj, t, x)
            v = x(4);
            u = obj.u_min_constant;
            % Note. this is hacking, we should be aware of dt, which is a
            % sampling time, to do this.
            dt = 0.01;
            if v <= (obj.v_min - dt * u(2)) && obj.state_dependent_input_bound
                u(2) = 0;
            end
        end
    end 
end