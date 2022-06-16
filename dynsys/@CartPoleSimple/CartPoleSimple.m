%% Author Jason Choi (jason.choi@berkeley.edu)
%% Reference, Tedrake, Underactuated Robotics
classdef CartPoleSimple < CtrlAffineSysFL
    %% Template class for the cart-pole type of systems.
    %% Uses the dynamics model in Tedrake et al., there is no damping in this model.
    properties
        A_sym % Symbolix expression for the Jacobian matrix at the origin
        A_origin % Jacobian Matrix at the origin
        B_origin % Jacobian Matrix at the origin
        l % Length of pendulum
        m % Mass of pendulum
        M % Mass of cart
        gravity
        J % Moment of inertia of the pendulum
    end
    
    methods
        function obj = CartPoleSimple(params, output_to_regulate, feedback_gain)
        % output_to_regulate: 'theta', 's'
        %   'theta': regulate angle of the pendulum (default)
        %   's': regulate position of the cart.
        if nargin < 2
            output_to_regulate = 'theta';
        end
        if nargin < 3            
            feedback_gain = [9, 6];
        end
            params.output_to_regulate = output_to_regulate;
            params.K_siso = feedback_gain;
            params.rel_deg_y = 2;
            obj@CtrlAffineSysFL(params, 'symbolic', 'siso');
            obj.l = params.l;
            obj.m = params.m;
            obj.M = params.M;
            obj.gravity = params.g;
            if isfield(params, 'J')
                obj.J = params.J;
            else
                obj.J = (1/3) * obj.m * obj.l^2;
            end
        end
        
        function [x, f, g] = defineSystem(obj, params)
            l = params.l;  % [m]      length of pendulum
            m = params.m;  % [kg]     mass of pendulum
            M = params.M;  % [kg]     mass of cart
            gravity = params.g; % [m/s^2]  acceleration of gravity
            if isfield(params, 'J')
                J = params.J;
            else
                J = (1/3) * m * l^2;
            end
            
            syms s ds theta dtheta real
            % theta: angle of the rod, upright is 0, in the
            % counter-clockwise direction.
            % s: position of the cart. (positive in the direction to right.)
            % u: input force on the cart.
            x = [theta; s; dtheta; ds;];

            matrix_inertia = [m*l^2+J, -m*l*cos(theta);
                               -m*l*cos(theta), m+M];
            ddq_drift = matrix_inertia\[m*gravity*l*sin(theta);-m*l*dtheta^2*sin(theta)];

            f = [dtheta;
                ds;
                ddq_drift];    

            g = [0;
                0;
                matrix_inertia\[0;1]];
        end
        
        function [y, z] = defineOutputWithZeroCoords(obj, params, x)
            if strcmp(params.output_to_regulate, 'theta')
                y = x(1);
                z = [x(2); x(4)*cos(x(1))-x(3)];
            elseif strcmp(params.output_to_regulate, 's')
                y = x(2);
                z = [x(1); x(4)*cos(x(1))-x(3)];
            end
        end    
          
        function p = get_linear_momentum(obj, x)
            p = obj.M * x(4) + obj.m * (x(4) - obj.l * cos(x(1)) * x(3));
        end
    end
end
