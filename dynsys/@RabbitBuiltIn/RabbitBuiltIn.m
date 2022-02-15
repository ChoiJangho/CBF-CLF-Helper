%% Main Reference
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

classdef RabbitBuiltIn < CtrlAffineSysFL
    properties
        reset_map_matrix = [1, 0, 0, 0, 0, 0, 0;
                             0, 1, 0, 0, 0, 0, 0;
                             0, 0, 1, 0, 0, 0, 0;
                             0, 0, 0, 0, 0, 1, 0;
                             0, 0, 0, 0, 0, 0, 1;
                             0, 0, 0, 1, 0, 0, 0;
                             0, 0, 0, 0, 1, 0, 0];
        fall_threshold = 0.699;
        theta_init
        theta_end
        a_bez
        
        %% Used for stepping stone scenario
        step_min = [];
        step_max = [];
        steps_min = [];
        steps_max = [];
        step_index = [];
        default_step_width = 0.05;
        gamma_b
    end
    methods
        function obj = RabbitBuiltIn(params)
            % Always using built-in option for setup.
            obj = obj@CtrlAffineSysFL(params, 'built-in');
            obj.theta_init = obj.params.theta_init;
            obj.theta_end = obj.params.theta_end;
            obj.a_bez = obj.params.a_bez;
            if isfield(params, 'gamma_b')
                obj.gamma_b = obj.params.gamma_b;
                obj.n_cbf = 2;
            else
                obj.n_cbf = 0;
            end
            if isfield(params, 'steps_min')
                obj.steps_min = params.steps_min;
                if isfield(params, 'steps_max')
                    obj.steps_max = params.steps_max;
                else
                    obj.steps_max = obj.steps_min + obj.default_step_width;                    
                end
                obj.step_min = obj.steps_min(1);
                obj.step_max = obj.steps_max(1);
                obj.step_index = 1;
            elseif isfield(params, 'step_min')
                obj.step_min = params.step_min;
                if isfield(params, 'step_max')
                    obj.step_max = params.stes_max;
                else
                    obj.step_max = obj.step_min + obj.default_step_width;                    
                end                
            end            
            obj.n_cbf = 2;
        end
        
        function [f, g] = defineDynamics(obj, x)
             n = obj.xdim / 2;

            q = x(1:n); 
            dq = x(n+1:2*n);

            [D, C, G, B] = obj.get_dynamics_matrices(x);
            JSt = J_RightToe(q);
            JSt = JSt([1,3],:);
            dJSt = dJ_RightToe(q,dq);
            dJSt = dJSt([1,3],:);

            H = C*dq + G;
            % Constraint Force to enforce the holonomic constraint:
            FSt_u = - pinv(JSt*(D\JSt'))*(JSt*(D\B));
            FSt_nu = - pinv(JSt*(D\JSt'))*(JSt*(D\(-H)) + dJSt*dq);

            f = [dq; D\(-C*dq - G)];
            g1 = [zeros(length(dq),obj.udim); D\B];
            g2 = [zeros(length(dq),size(FSt_nu ,1)); D\JSt'];

            % dx = f(x)+g(x)u of nominal model
            f = f + g2 * FSt_nu;
            g = g1 + g2 * FSt_u;
        end
        function f_ = f(obj, x)
            % RABBIT f(x)
            % TODO:multiple calculation of prior procedures => if we
            % maintain this architecture of different function, it is
            % inevitable or else, we can have another prior function but
            % still we need to waste memory space!
            % memory <-> time complexity issue
            [f_,~] = obj.defineDynamics(x);
        end
        function g_ = g(obj, x)
            % RABBIT g(x)
            [~, g_] = obj.defineDynamics(x);
        end
        function zs = z(obj, xs)
            % zero dynamics
            n = obj.xdim/2;
            q = xs(1:n, :);
            dq = xs(n+1:2*n, :);
            
            theta = q(3, :) + q(4, :) + q(5, :)/2;            
            dtheta = dq(3, :) + dq(4, :) + dq(5, :)/2;
            
            zs = [theta; dtheta];
        end
        
        function y_ = y(obj, x)
            % TODO: Rabbit's output
            n = obj.xdim / 2;
            q = x(1:n);
            theta = q(3) + q(4) + q(5)/2;
            
            phase = min(max((theta - obj.theta_init)/(obj.theta_end - obj.theta_init)));
            
            yd = obj.bezier_outputs(obj.a_bez, phase); % Not very structured actually
            ya = q(4:end);
            y_ = ya-yd;
        end
        
%         function phase_ = phase(obj, x)
%             % TODO: Rabbit's phase => redundant(?)
%         end
        
        function Lfy = lf_y(obj, x)
            % TODO: Not used in RABBIT
            n = obj.xdim / 2;
            q = x(1:n);
            dq = x(n+1: 2*n);
            
            dydq_des = obj.dydq_d_gen(q, obj.a_bez, obj.theta_init, obj.theta_end); %[4 x 7]
            dydq_act = obj.dydq_a_gen(q);
            
            Lfy = (dydq_act - dydq_des)*dq; % Use dy = Lfy + Lgyu = Lfy property
        end
        
        function Lgy = lg_y(obj, x)
            Lgy = zeros(obj.ydim, 1);
        end
        
        function LgLfy = lglf_y(obj, x)
            % TODO: Rabbit's lie derivative of LgLfy
            n = obj.xdim / 2;
            q = x(1:n);
            
            dydq_des = obj.dydq_d_gen(q, obj.a_bez, obj.theta_init, obj.theta_end); %[4 x 7]
            dydq_act = obj.dydq_a_gen(q);
            g_s = obj.g(x);
            
            LgLfy = (dydq_act-dydq_des)*g_s(n+1:end, :);
        end
        
        function L2fy = l2f_y(obj, x)
            % TODO: Rabbit's lie derivative of L2fy
            n = obj.xdim/2;
            q = x(1:n);
            dq = x(n+1:2*n);
                        
            L2ydq_des = obj.L2ydq_d_gen(q, dq, obj.a_bez, obj.theta_init, obj.theta_end);
            L2ydq_act = obj.L2ydq_a_gen(q, dq);
            f_s = obj.f(x);
            
            L2fy = (L2ydq_act - L2ydq_des)*f_s;
        end
        
        function [D, C, G, B] = get_dynamics_matrices(obj, x)
            %% [D, C, G, B] = get_dynamics_matrices(obj, x)
            % D: Mass-Intertia Matrix
            % C: Coriollis Matrix
            % G: Gravity Vector
            % B: Input Mapping Matrix
            n = obj.xdim / 2;
            q = x(1:n); 
            dq = x(n+1:2*n);
            
            D = obj.D_gen_rel(q);
            C = obj.C_gen_rel(q,dq); % 7 x 7
            G = obj.G_gen_rel(q);
            B =[0 0 0 0 
                 0 0 0 0
                 0 0 0 0
                 1 0 0 0
                 0 1 0 0
                 0 0 1 0
                 0 0 0 1];
        end
    end
end

