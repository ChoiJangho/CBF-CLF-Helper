classdef QuanserCartPole < CtrlAffineSys
    methods
        function obj = QuanserCartPole(params, varargin)
            obj = obj@CtrlAffineSys(params, varargin{:});            
        end
        function [s, f_vec, g_vec] = defineSystem(obj, params)
            % params:
                % IP02_LOAD_TYPE: 'NO_LOAD', 'WEIGHT'
                % PEND_TYPE: 'LONG_24IN', 'MEDIUM_12IN'
                
            % theta: heading, sigma: yaw rate
            syms x theta x_dot theta_dot
            
            s = [x; theta; x_dot; theta_dot];
            
            % Gravity
            g = 9.81;
            
            [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq ] = ...
                obj.config_ip02( params.IP02_LOAD_TYPE );
            
            [ Mp, Lp, lp, Jp, Bp ] = obj.config_sp( params.PEND_TYPE );  
            
            % Lumped Mass of the Cart System (accounting for the rotor inertia)
            Jeq = Mc + eta_g * Kg^2 * Jm / r_mp^2;
            
            % TODO: FINISH THIS DYNAMICS
            % Dynamics computation
            costheta = -cos(theta);
            sintheta = -sin(theta);

            fc = eta_g*Kg*Kt/(Rm*r_mp)*(-Kg*Km*x_dot/r_mp);
            gc =  eta_g*Kg*Kt/(Rm*r_mp)* eta_m;

            D = [Jeq+Mp, Mp*lp*costheta ;
                Mp*lp*costheta, Jp+Mp*lp^2];
            Cdq_g = [-Mp*lp*sintheta*theta_dot^2+Beq*x_dot; Mp*lp*g*sintheta+Bp*theta_dot];
            B = [1;0];
            
            f_vec = [x_dot; theta_dot; D\ ((B*fc) - Cdq_g)];
            g_vec = [0; 0; D\(B*gc)];
            
        end
        
        function [ Mp, Lp, lp, Jp, Bp ] = config_sp(obj, PEND_TYPE )
            % from Inch to Meter
            K_IN2M = 0.0254;
            % Set these variables (used in Simulink Diagrams)
            if strcmp( PEND_TYPE, 'LONG_24IN')
                % Pendulum Mass (with T-fitting)
                Mp = 0.230;
                % Pendulum Full Length (with T-fitting, from axis of rotation to tip)
                Lp = ( 25 + 1 / 4 ) * K_IN2M;  % = 0.6413;
                % Distance from Pivot to Centre Of Gravity
                lp = 13 * K_IN2M;  % = 0.3302
                % Pendulum Moment of Inertia (kg.m^2) - approximation
                Jp = Mp * Lp^2 / 12;  % = 7.8838 e-3
                % Equivalent Viscous Damping Coefficient (N.m.s/rad)
                Bp = 0.0024;
            elseif strcmp( PEND_TYPE, 'MEDIUM_12IN')
                % Pendulum Mass (with T-fitting)
                Mp = 0.127;
                % Pendulum Full Length (with T-fitting, from axis of rotation to tip)
                Lp = ( 13 + 1 / 4 ) * K_IN2M;  % = 0.3365
                % Distance from Pivot to Centre Of Gravity
                lp = 7 * K_IN2M;  % = 0.1778
                % Pendulum Moment of Inertia (kg.m^2) - approximation
                Jp = Mp * Lp^2 / 12;  % = 1.1987 e-3
                % Equivalent Viscous Damping Coefficient (N.m.s/rad)
                Bp = 0.0024;
            else 
                error( 'Error: Set the type of pendulum.' )
            end
        end
        
        function [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, M, r_mp, Beq] = ...
                config_ip02(obj, IP02_LOAD_TYPE)
            % from Inch to Meter
            K_IN2M = 0.0254;
            % from rad/s to RPM
            K_RDPS2RPM = 60 / ( 2 * pi );
            % from oz-force to N
            K_OZ2N = 0.2780139;
            
            % Motor Armature Resistance (Ohm)
            Rm = 2.6;
            % Motor Torque Constant (N.m/A)
            Kt = 1.088 * K_OZ2N * K_IN2M; % = .00767
            % Motor ElectroMechanical Efficiency [ = Tm * w / ( Vm * Im ) ]
            eta_m = 1;
            % Motor Back-EMF Constant (V.s/rad)
            Km = 0.804e-3 * K_RDPS2RPM; % = .00767
            % Rotor Inertia (kg.m^2)
            Jm = 5.523e-5 * K_OZ2N * K_IN2M; % = 3.9e-7
            % IP02 Cart Mass, with 3 cable connectors (kg)
            Mc2 = 0.57;
            % Cart Weight Mass (kg)
            Mw = 0.37;
            % Planetary Gearbox (a.k.a. Internal) Gear Ratio
            Kg = 3.71;
            % Planetary Gearbox Efficiency
            eta_g = 1;
            % Motor Pinion Radius (m)
            r_mp = 0.5 / 2 * K_IN2M;  %  = 6.35e-3
            if strcmp( IP02_LOAD_TYPE, 'NO_LOAD')
                M = Mc2;
                Beq = 4.3;
            elseif strcmp ( IP02_LOAD_TYPE, 'WEIGHT')
                M = Mc2 + Mw;
                Beq = 5.4;
            else 
                error( 'Error: Set the IP02 load configuration.' )
            end
        end
        
        function clf = defineClf(obj, params, x_sym)
            x = x_sym;
            [~, f_, g_] = obj.defineSystem(params);
            A_sym = jacobian(f_, x);
            A = double(subs(A_sym, x, zeros(4, 1)));
            B = double(subs(g_, x, zeros(4, 1)));

%             Q = eye(obj.xdim);
            Q = diag([100,100,0.01,0.01]);
            R = obj.udim;
            P = icare(A, B, Q, R);
            clf = x' * P * x;
%             disp(P)
        end
        
        function cbf = defineCbf(obj, params, x_sym)
            x = x_sym(1);
            x_dot = x_sym(3);
            cbf = -2*x*x_dot + params.k_cbf*(params.x_lim^2 - x^2);
        end
    end
end
