%% Author Jason Choi (jason.choi@berkeley.edu)
%% Reference, Tedrake, Underactuated Robotics
classdef CartPole < CtrlAffineSys
    %% Template class for the cart-pole type of systems.
    %% Uses the dynamics model in Tedrake et al., there is no damping in this model.
    properties
        %% Main Model parameters
        l % Length of pendulum (pivot to the center of gravity)
        m % Mass of pendulum
        M % Mass of cart
        gravity
        J % Moment of inertia of the pendulum
        L % Length of pendulum (pivot to the end of the rod, for visualization)
        b_cart % drag coefficient w.r.t. cart movement.
        b_pole % drag coefficient w.r.t. pole movement.
        
        Jeq % Equivalent mass of the cart (accounting for the rotor inertia)
            % Used to evaluate the kinetic energy and the momentum of the cart.        
        
        A_sym % Symbolix expression for the Jacobian matrix at the origin
        A_origin % Jacobian Matrix at the origin
        B_origin % Jacobian Matrix at the origin                
    end
    
    methods
        function obj = CartPole(params)
            if ~isfield(params, 'gravity')
                params.gravity = 9.81;
            end            
            if ~isfield(params, 'J')
                params.J = (1/3) * params.m * params.l^2;
            end
            if ~isfield(params, 'L')
                params.L = 2 * params.l;
            end
            if ~isfield(params, 'Jeq')
                params.Jeq = params.M;
            end
            if ~isfield(params, 'b_cart')
                params.b_cart = 0;
            end
            if ~isfield(params, 'b_pole')
                params.b_pole = 0;
            end
            if ~isfield(params, 'u_max') && ~isfield(params, 'u_min')
                params.u_max = (params.m + params.M) * 10;
                % This is roughly limiting the maximal acceleration of the
                % cart to be 10 m/s^2.
                params.u_min = -params.u_max;
            elseif ~isfield(params, 'u_min')
                params.u_min = -params.u_max;
            elseif ~isfield(params, 'u_max')
                params.u_max = -params.u_min;
            end
                        
            params.dims_angle = [0, 1, 0, 0]; % the second state is the angle variable.
            
            obj@CtrlAffineSys(params, 'symbolic');
            obj.l = params.l;
            obj.m = params.m;
            obj.M = params.M;            
            obj.gravity = params.gravity;
            obj.J = params.J;
            obj.L = params.L;
            obj.Jeq = params.Jeq;
            obj.b_cart = params.b_cart;
            obj.b_pole = params.b_pole;            
        end
        
        
        function [x, f, g] = defineSystem(obj, params) 
            syms s ds theta dtheta real
            % theta: angle of the rod, upright is 0, in the
            % counter-clockwise direction.
            % s: position of the cart. (positive in the direction to right.)
            % u: input force on the cart.
            x = [s; theta; ds; dtheta;];
            q = [s; theta];
            dq = [ds; dtheta];
            q_act = s;
            V = obj.get_potential_energy(params, x);
            T = obj.get_kinetic_energy(params, x);
            [D, C, G, B] = get_lagrangian_dynamics(T, V, q, dq, q_act);
            B_drag = [params.b_cart * ds; params.b_pole * dtheta];
            f = [dq; D\(-C * dq - G - B_drag)];
            g = [zeros(2, 1); D\B];            
        end                       
        function E = total_energy(obj, x)
            V = obj.get_potential_energy(obj.params, x);
            T = obj.get_kinetic_energy(obj.params, x);
            E = V + T;
        end
        function E = pole_energy(obj, x)
            V = obj.get_potential_energy(obj.params, x);
            T = obj.get_kinetic_energy_pole(obj.params, x);
            E = V + T;
        end
        function E = potential_energy_upright(obj)
            E = obj.m * obj.gravity * obj.l;
        end
        
        function P = linear_momentum(obj, x)
            P = obj.get_linear_momentum(obj.params, x);
        end
    end
        
    methods(Static)
        function [x_pendulum_cg, y_pendulum_cg] = get_pendulum_cg_position(params, x)
            % x: state
            theta = x(2);
            x_cart = x(1);
            x_pendulum_cg = x_cart - params.l * sin(theta);
            y_pendulum_cg = params.l * cos(theta);
        end
        
        function V = get_potential_energy(params, x)
            % x: state
            % this can be either a numeric array or a symbolic vector.
            % 0 at the cart height.
            theta = x(2);
            V = params.m * params.gravity * params.l * cos(theta);
        end
        
        function T_cart = get_kinetic_energy_cart(params, x)
            ds = x(3);
            T_cart = 0.5 * params.Jeq * ds^2;            
        end
        
        function T_pole = get_kinetic_energy_pole(params, x)
            theta = x(2);
            dtheta = x(4);
            ds = x(3);
            T_pole_translation = 0.5 * params.m * (ds^2 + params.l^2 * dtheta^2) ...
                - params.m * params.l * cos(theta) * dtheta * ds;
            T_pole_rotation = 0.5 * params.J * dtheta^2;
            T_pole = T_pole_translation + T_pole_rotation;            
        end
        
        function T = get_kinetic_energy(params, x)
            T_cart = CartPole.get_kinetic_energy_cart(params, x);
            T_pole = CartPole.get_kinetic_energy_pole(params, x);                   
            T = T_cart + T_pole;
        end
         
        function p = get_linear_momentum(params, x)
            p = params.Jeq * x(3) + params.m * (x(3) - params.l * cos(x(2)) * x(4));
        end
    end
end
