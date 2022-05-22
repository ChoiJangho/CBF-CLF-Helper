classdef QuanserCartPole < CtrlAffineSys
    properties        
        a_max % Maximum acceleration of pivot (m/s^2)
        Er % Reference Energy (J)
        
        eta_g % Planetary Gearbox Efficiency
        eta_m % Motor ElectroMechanical Efficiency [ = Tm * w / ( Vm * Im ) ]

        Jeq  % Lumped Mass of the Cart System (accounting for the rotor inertia)
        Jp % Pendulum Moment of Inertia (kg.m^2) - approximation

        Kt % Motor Torque Constant (N.m/A)
        Kg % Planetary Gearbox (a.k.a. Internal) Gear Ratio
        Km % Motor Back-EMF Constant (V.s/rad)
        
        lp % Distance from Pivot to Centre Of Gravity
        Mp % Pendulum Mass (with T-fitting)        
        Mc % Cart Mass
        
        Rm % Motor Armature Resistance (Ohm)
        r_mp % Motor Pinion Radius (m)        
        
        gravity = 9.81;
    end
    methods
        function obj = QuanserCartPole(params, varargin)
            obj = obj@CtrlAffineSys(params, varargin{:});
            [obj.Rm, Jm, obj.Kt, obj.eta_m, obj.Km, obj.Kg, obj.eta_g, obj.Mc, obj.r_mp, Beq] = ...
                obj.config_ip02( params.IP02_LOAD_TYPE );
            [obj.Mp, Lp, obj.lp, obj.Jp, Bp ] = obj.config_sp( params.PEND_TYPE );  
            obj.Jeq = obj.Mc + obj.eta_g * obj.Kg^2 * Jm / obj.r_mp^2;
            [obj.Er, obj.a_max] = d_swing_up(obj.eta_m, obj.eta_g, ...
                obj.Kg, obj.Kt, obj.Rm, obj.r_mp, obj.Jeq, obj.Mp, obj.lp);            
        end
        function [s, f_vec, g_vec] = defineSystem(obj, params)
            % params:
                % IP02_LOAD_TYPE: 'NO_LOAD', 'WEIGHT'
                % PEND_TYPE: 'LONG_24IN', 'MEDIUM_12IN'
                
            syms x theta x_dot theta_dot
            
            s = [x; theta; x_dot; theta_dot];
            
            %% Set up model parameters from the quanser functions.
            % Gravity
            gravity = 9.81;            
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
            Cdq_g = [-Mp*lp*sintheta*theta_dot^2+Beq*x_dot; Mp*lp*gravity*sintheta+Bp*theta_dot];
            B = [1;0];
            
            f_vec = [x_dot; theta_dot; D\ ((B*fc) - Cdq_g)];
            g_vec = [0; 0; D\(B*gc)];
            
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
        
        function energy = get_total_energy(obj, x)
            % total energy is 0 at rest when theta = pi (downright.
            theta = x(2);
            dtheta = x(4);            
            energy = 0.5 * obj.Jp * dtheta^2 + obj.Mp * obj.lp * obj.gravity * (1 + cos(theta));            
        end
        
        function energy = get_total_energy2(obj, x)
            % total energy is 0 at rest when theta = 0 (upright).
            theta = x(2);
            dtheta = x(4);            
            energy = 0.5 * obj.Jp * dtheta^2 + obj.Mp * obj.lp * obj.gravity * (-1 + cos(theta)); 
        end        
        
        
        function draw_cart_pole(obj, x, theta, force, text1, text2)
            % theta: angle from below
            l = 1;
            xmin = -10; 
            xmax = 10;    
            height = 0.1;
            width  = 0.3;
            maxU = obj.u_max;
            if isempty(maxU)
                maxU = 10;
            end
            

            % Compute positions 
            cart = [ x + width,  height
                     x + width, -height
                     x - width, -height
                     x - width,  height
                     x + width,  height ];
            pendulum = [x, 0; x+2*l*sin(theta), -cos(theta)*2*l];


            clf; hold on
            plot(0,2*l,'k+','MarkerSize',20,'linewidth',2)
            plot([xmin, xmax], [-height-0.03, -height-0.03],'k','linewidth',2)

            % Plot force
            plot([0 force/maxU*xmax],[-0.3, -0.3],'g','linewidth',10)

            % Plot the cart-pole
            fill(cart(:,1), cart(:,2),'k','edgecolor','k');
            plot(pendulum(:,1), pendulum(:,2),'r','linewidth',4)

            % Plot the joint and the tip
            plot(x,0,'y.','markersize',24)
            plot(pendulum(2,1),pendulum(2,2),'y.','markersize',24)

            % Text
            text(0,-0.3,'applied force')
            % text(0,-0.5,'immediate reward')
            if exist('text1','var')
              text(0,-0.9, text1)
            end
            if exist('text2','var')
              text(0,-1.1, text2)
            end

            set(gca,'DataAspectRatio',[1 1 1],'XLim',[xmin xmax],'YLim',[-2 2]);
            axis off;
            drawnow;
        end
        
        function fig = draw_rollout(obj, xs, us, ts, dt, title_text)
            fig = figure;
            for r = 1:length(ts)-1
                obj.draw_cart_pole(xs(1, r), pi-xs(2, r), us(r),  ...
                      ['t=' num2str(r*dt) ' sec'], title_text);
                pause(dt);
            end
        end
        
        function p = get_linear_momentum(obj, x)
            p = obj.Mc * x(3) + obj.Mp * (x(3) + 0.5 * obj.lp * cos(x(2)) * x(4));
        end
    end
    
    methods(Static)
        %% These methods are provided by Quanser.
        function [ Mp, Lp, lp, Jp, Bp ] = config_sp(PEND_TYPE)
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
        
        function [ Rm, Jm, Kt, eta_m, Km, Kg, eta_g, Mc, r_mp, Beq] = ...
                config_ip02(IP02_LOAD_TYPE)
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
                Mc = Mc2;
                Beq = 4.3;
            elseif strcmp ( IP02_LOAD_TYPE, 'WEIGHT')
                Mc = Mc2 + Mw;
                Beq = 5.4;
            elseif strcmp ( IP02_LOAD_TYPE, '2WEIGHTS')
                Mc = Mc2 + 2*Mw;
                Beq = 6.5;
            else 
                error( 'Error: Set the IP02 load configuration.' )
            end
        end

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
