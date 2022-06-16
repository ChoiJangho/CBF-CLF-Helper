%% Main reference: Student Workbook - Linear Pendulum Gantry Experiment, Quanser

classdef QuanserCartPole < CartPole
    properties        
        constant_V2F % Motor voltage to cart force ratio 
        constant_motor_drag % Motor voltage to cart force ratio         
    end
    methods
        function obj = QuanserCartPole(params, IP02_LOAD_TYPE, PEND_TYPE)
            if nargin < 2
                IP02_LOAD_TYPE = 'NO_LOAD';
            end
            if nargin < 3
                PEND_TYPE = 'LONG_24IN';
            end                
            [ Mp, Lp, lp, Jp, Bp ] = QuanserCartPole.config_sp(PEND_TYPE);
            [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
                    QuanserCartPole.config_ip02(IP02_LOAD_TYPE);
                            
            params.l = lp;
            params.J = Jp;
            params.m = Mp;
            params.M = Mc;
            params.gravity = 9.81;
            params.L = Lp;            
            params.Jeq = Mc + eta_g * Kg^2 * Jm / r_mp^2;            
            params.b_pole = Bp;
            params.b_cart = Beq;
            
            % Equivalent cart force (eq (2.11) in the manual): Fc = constant_V2F * u - constant_motor_drag * ds
            % Motor voltage to cart force ratio 
            params.constant_V2F= eta_g * Kg * Kt * eta_m / (Rm * r_mp);
            % Drag to cart induced by the motor
            params.constant_motor_drag = eta_g * Kg^2 * Kt * Km / (Rm * r_mp^2);            
            
            % Motor voltage limit: fyi, strict hardware limit is 10.
            params.u_max = 8;
            params.u_min = -8;
            
            % For Debug
            obj = obj@CartPole(params);
            obj.constant_V2F = params.constant_V2F;
            obj.constant_motor_drag = params.constant_motor_drag;
        end
    
        function clf = defineClf(obj, params, x)
            [~, f_, g_] = obj.defineSystem(params);
            A_sym = jacobian(f_, x);
            A = double(subs(A_sym, x, zeros(4, 1)));
            B = double(subs(g_, x, zeros(4, 1)));

%             Q = eye(obj.xdim);
            Q = diag([100,100,0.01,0.01]);
            R = obj.udim;
            P = icare(A, B, Q, R);
            clf = x' * P * x;
        end
        
        function cbf = defineCbf(obj, params, x_sym)
            x = x_sym(1);
            x_dot = x_sym(3);
            cbf = -2*x*x_dot + params.k_cbf*(params.x_lim^2 - x^2);
        end        
    end
    
    methods(Static)
        [ Mp, Lp, lp, Jp, Bp ] = config_sp(PEND_TYPE)
        [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
            config_ip02(IP02_LOAD_TYPE)
        
        function [Er, a_max] = d_swing_up(eta_m, eta_g, Kg, Kt, Rm, r_mp, Jeq, Mp, lp)
            % Reference Energy (J)
            Er = 2*Mp*lp*9.81;
            % Maximum force for 5 V
            Fc_max = (eta_m*eta_g*Kg*Kt*5)/(Rm*r_mp);
            % Maximum acceleration of pivot (m/s^2)
            a_max = (Fc_max / Jeq);
        end        
    end            
end
