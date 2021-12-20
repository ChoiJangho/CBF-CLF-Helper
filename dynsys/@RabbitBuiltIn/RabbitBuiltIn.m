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
    end
    methods
        function obj = RabbitBuiltIn(params)
            % Always using built-in option for setup.
            obj = obj@CtrlAffineSysFL(params, 'built-in');
        end
        
        function [f, g] = defineDynamics(obj, x)
             n = obj.xdim / 2;

            q = x(1:n); 
            dq = x(n+1:2*n);

            %[D, C, G, B] = obj.gen_DCGB_rel(q);
            %D = D_gen_rel(q, obj.params.scale, obj.params.torso_add); % 7 x 7
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
%             theta_init = obj.params.legacy.theta_init;
%             theta_end = obj.params.legacy.theta_end;
%             phase = min(max((theta - theta_init)/(theta_end - theta_init)));
            
            dtheta = dq(3, :) + dq(4, :) + dq(5, :)/2;
            
            zs = [theta; dtheta];
        end
        function B = cbf(obj, x)
            % TODO: CBF(x)
        end
        function LfB = lf_cbf(obj, x)
            % TODO: LfB(x)
        end
        function LgB = lg_cbf(obj, x)
            % TODO: LgB(x)
        end
        
        function y_ = y(obj, x)
            % TODO: Rabbit's output
            n = obj.xdim / 2;
            q = x(1:n);
            theta = q(3) + q(4) + q(5)/2;
            
            a_bez = obj.params.legacy.a_bez;
            theta_init = obj.params.legacy.theta_init;
            theta_end = obj.params.legacy.theta_end;
            phase = min(max((theta - theta_init)/(theta_end - theta_init)));
            
            yd = obj.bezier_outputs(a_bez, phase); % Not very structured actually
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
            
            a_bez = obj.params.legacy.a_bez;
            theta_init = obj.params.legacy.theta_init;
            theta_end = obj.params.legacy.theta_end;
            
            dydq_des = obj.dydq_d_gen(q, a_bez, theta_init, theta_end); %[4 x 7]
            dydq_act = obj.dydq_a_gen(q);
            
            Lfy = (dydq_act - dydq_des)*dq; % Use dy = Lfy + Lgyu = Lfy property
        end
        
        function Lgy = lg_y(obj, x)
            % TODO: Not used in RABBIT
            Lgy = zeros(obj.ydim, 1);
        end
        
        function LgLfy = lglf_y(obj, x)
            % TODO: Rabbit's lie derivative of LgLfy
            n = obj.xdim / 2;
            q = x(1:n);
            
            a_bez = obj.params.legacy.a_bez;
            theta_init = obj.params.legacy.theta_init;
            theta_end = obj.params.legacy.theta_end;
            
            dydq_des = obj.dydq_d_gen(q, a_bez, theta_init, theta_end); %[4 x 7]
            dydq_act = obj.dydq_a_gen(q);
            g_s = obj.g(x);
            
            LgLfy = (dydq_act-dydq_des)*g_s(n+1:end, :);
        end
        
        function L2fy = l2f_y(obj, x)
            % TODO: Rabbit's lie derivative of L2fy
            n = obj.xdim/2;
            q = x(1:n);
            dq = x(n+1:2*n);
            
            a_bez = obj.params.legacy.a_bez;
            theta_init = obj.params.legacy.theta_init;
            theta_end = obj.params.legacy.theta_end;
            
            L2ydq_des = obj.L2ydq_d_gen(q, dq, a_bez, theta_init, theta_end);
            L2ydq_act = obj.L2ydq_a_gen(q, dq);
            f_s = obj.f(x);
            
            L2fy = (L2ydq_act - L2ydq_des)*f_s;
        end
        
        %% Generator
        function dydq_a = dydq_a_gen(obj,in1)
            %DYDQ_A_GEN
            %    DYDQ_A = DYDQ_A_GEN(IN1)

            %    This function was generated by the Symbolic Math Toolbox version 8.0.
            %    11-Dec-2019 16:46:17

            dydq_a = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,7]);
        end     
        function dydq_d = dydq_d_gen(obj,in1,in2,th0,thf)
            %DYDQ_D_GEN
            %    DYDQ_D = DYDQ_D_GEN(IN1,IN2,TH0,THF)

            %    This function was generated by the Symbolic Math Toolbox version 8.0.
            %    11-Dec-2019 16:46:04

            a01 = in2(1);
            a02 = in2(2);
            a03 = in2(3);
            a04 = in2(4);
            a11 = in2(5);
            a12 = in2(6);
            a13 = in2(7);
            a14 = in2(8);
            a21 = in2(9);
            a22 = in2(10);
            a23 = in2(11);
            a24 = in2(12);
            a31 = in2(13);
            a32 = in2(14);
            a33 = in2(15);
            a34 = in2(16);
            a41 = in2(17);
            a42 = in2(18);
            a43 = in2(19);
            a44 = in2(20);
            a51 = in2(21);
            a52 = in2(22);
            a53 = in2(23);
            a54 = in2(24);
            q1 = in1(3,:);
            q31 = in1(4,:);
            q32 = in1(5,:);
            t6 = q32.*(1.0./2.0);
            t2 = q1+q31+t6-th0;
            t3 = t2.^2;
            t4 = th0-thf;
            t5 = 1.0./t4.^5;
            t7 = t3.^2;
            t8 = 1.0./t4;
            t11 = t2.*t8;
            t9 = t11+1.0;
            t10 = t9.^2;
            t12 = t10.^2;
            t13 = 1.0./t4.^3;
            t14 = 1.0./t4.^2;
            t15 = 1.0./t4.^4;
            t16 = a41.*t5.*t7.*5.0;
            t17 = a01.*t8.*t12.*5.0;
            t18 = q1.*2.0;
            t19 = q31.*2.0;
            t25 = th0.*2.0;
            t20 = q32+t18+t19-t25;
            t21 = a21.*t9.*t10.*t14.*t20.*1.0e1;
            t22 = a21.*t3.*t10.*t13.*3.0e1;
            t23 = a41.*t2.*t3.*t9.*t15.*2.0e1;
            t24 = t16+t17+t21+t22+t23-a11.*t8.*t12.*5.0-a51.*t5.*t7.*5.0-a31.*t3.*t10.*t13.*3.0e1-a11.*t2.*t9.*t10.*t14.*2.0e1-a31.*t2.*t3.*t9.*t15.*2.0e1;
            t26 = a42.*t5.*t7.*5.0;
            t27 = a02.*t8.*t12.*5.0;
            t28 = a22.*t9.*t10.*t14.*t20.*1.0e1;
            t29 = a22.*t3.*t10.*t13.*3.0e1;
            t30 = a42.*t2.*t3.*t9.*t15.*2.0e1;
            t31 = t26+t27+t28+t29+t30-a12.*t8.*t12.*5.0-a52.*t5.*t7.*5.0-a32.*t3.*t10.*t13.*3.0e1-a12.*t2.*t9.*t10.*t14.*2.0e1-a32.*t2.*t3.*t9.*t15.*2.0e1;
            t32 = a43.*t5.*t7.*5.0;
            t33 = a03.*t8.*t12.*5.0;
            t34 = a23.*t9.*t10.*t14.*t20.*1.0e1;
            t35 = a23.*t3.*t10.*t13.*3.0e1;
            t36 = a43.*t2.*t3.*t9.*t15.*2.0e1;
            t37 = t32+t33+t34+t35+t36-a13.*t8.*t12.*5.0-a53.*t5.*t7.*5.0-a33.*t3.*t10.*t13.*3.0e1-a13.*t2.*t9.*t10.*t14.*2.0e1-a33.*t2.*t3.*t9.*t15.*2.0e1;
            t38 = a44.*t5.*t7.*5.0;
            t39 = a04.*t8.*t12.*5.0;
            t40 = a24.*t9.*t10.*t14.*t20.*1.0e1;
            t41 = a24.*t3.*t10.*t13.*3.0e1;
            t42 = a44.*t2.*t3.*t9.*t15.*2.0e1;
            t43 = t38+t39+t40+t41+t42-a14.*t8.*t12.*5.0-a54.*t5.*t7.*5.0-a34.*t3.*t10.*t13.*3.0e1-a14.*t2.*t9.*t10.*t14.*2.0e1-a34.*t2.*t3.*t9.*t15.*2.0e1;
            dydq_d = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t24,t31,t37,t43,t24,t31,t37,t43,a01.*t8.*t12.*(5.0./2.0)-a11.*t8.*t12.*(5.0./2.0)+a41.*t5.*t7.*(5.0./2.0)-a51.*t5.*t7.*(5.0./2.0)+a21.*t3.*t10.*t13.*1.5e1-a31.*t3.*t10.*t13.*1.5e1-a11.*t2.*t9.*t10.*t14.*1.0e1+a21.*t2.*t9.*t10.*t14.*1.0e1-a31.*t2.*t3.*t9.*t15.*1.0e1+a41.*t2.*t3.*t9.*t15.*1.0e1,a02.*t8.*t12.*(5.0./2.0)-a12.*t8.*t12.*(5.0./2.0)+a42.*t5.*t7.*(5.0./2.0)-a52.*t5.*t7.*(5.0./2.0)+a22.*t3.*t10.*t13.*1.5e1-a32.*t3.*t10.*t13.*1.5e1-a12.*t2.*t9.*t10.*t14.*1.0e1+a22.*t2.*t9.*t10.*t14.*1.0e1-a32.*t2.*t3.*t9.*t15.*1.0e1+a42.*t2.*t3.*t9.*t15.*1.0e1,a03.*t8.*t12.*(5.0./2.0)-a13.*t8.*t12.*(5.0./2.0)+a43.*t5.*t7.*(5.0./2.0)-a53.*t5.*t7.*(5.0./2.0)+a23.*t3.*t10.*t13.*1.5e1-a33.*t3.*t10.*t13.*1.5e1-a13.*t2.*t9.*t10.*t14.*1.0e1+a23.*t2.*t9.*t10.*t14.*1.0e1-a33.*t2.*t3.*t9.*t15.*1.0e1+a43.*t2.*t3.*t9.*t15.*1.0e1,a04.*t8.*t12.*(5.0./2.0)-a14.*t8.*t12.*(5.0./2.0)+a44.*t5.*t7.*(5.0./2.0)-a54.*t5.*t7.*(5.0./2.0)+a24.*t3.*t10.*t13.*1.5e1-a34.*t3.*t10.*t13.*1.5e1-a14.*t2.*t9.*t10.*t14.*1.0e1+a24.*t2.*t9.*t10.*t14.*1.0e1-a34.*t2.*t3.*t9.*t15.*1.0e1+a44.*t2.*t3.*t9.*t15.*1.0e1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[4,7]);
        end
        function L2ydq_d = L2ydq_d_gen(obj,in1,in2,in3,th0,thf)
            %L2YDQ_D_GEN
            %    L2YDQ_D = L2YDQ_D_GEN(IN1,IN2,IN3,TH0,THF)

            %    This function was generated by the Symbolic Math Toolbox version 8.0.
            %    11-Dec-2019 16:46:17

            a01 = in3(1);
            a02 = in3(2);
            a03 = in3(3);
            a04 = in3(4);
            a11 = in3(5);
            a12 = in3(6);
            a13 = in3(7);
            a14 = in3(8);
            a21 = in3(9);
            a22 = in3(10);
            a23 = in3(11);
            a24 = in3(12);
            a31 = in3(13);
            a32 = in3(14);
            a33 = in3(15);
            a34 = in3(16);
            a41 = in3(17);
            a42 = in3(18);
            a43 = in3(19);
            a44 = in3(20);
            a51 = in3(21);
            a52 = in3(22);
            a53 = in3(23);
            a54 = in3(24);
            dq1 = in2(3,:);
            dq31 = in2(4,:);
            dq32 = in2(5,:);
            q1 = in1(3,:);
            q31 = in1(4,:);
            q32 = in1(5,:);
            t6 = q32.*(1.0./2.0);
            t2 = q1+q31+t6-th0;
            t3 = t2.^2;
            t4 = th0-thf;
            t5 = 1.0./t4.^5;
            t10 = 1.0./t4;
            t11 = t2.*t10;
            t7 = t11+1.0;
            t8 = t7.^2;
            t9 = 1.0./t4.^2;
            t12 = 1.0./t4.^3;
            t13 = q1.*2.0;
            t14 = q31.*2.0;
            t17 = th0.*2.0;
            t15 = q32+t13+t14-t17;
            t16 = 1.0./t4.^4;
            t18 = a41.*t2.*t3.*t5.*4.0e1;
            t19 = a01.*t7.*t8.*t9.*2.0e1;
            t20 = a21.*t7.*t8.*t9.*2.0e1;
            t21 = a21.*t8.*t12.*t15.*6.0e1;
            t22 = a21.*t3.*t7.*t16.*6.0e1;
            t23 = a41.*t3.*t7.*t16.*6.0e1;
            t24 = a41.*t2.*t3.*t5.*2.0e1;
            t25 = a01.*t7.*t8.*t9.*1.0e1;
            t26 = a21.*t7.*t8.*t9.*1.0e1;
            t27 = a21.*t8.*t12.*t15.*1.5e1;
            t28 = a21.*t2.*t8.*t12.*3.0e1;
            t29 = a21.*t3.*t7.*t16.*3.0e1;
            t30 = a41.*t3.*t7.*t16.*3.0e1;
            t40 = a31.*t2.*t3.*t5.*1.0e1;
            t41 = a51.*t2.*t3.*t5.*1.0e1;
            t42 = a11.*t7.*t8.*t9.*2.0e1;
            t43 = a11.*t2.*t8.*t12.*3.0e1;
            t44 = a31.*t3.*t7.*t16.*6.0e1;
            t31 = t24+t25+t26+t27+t28+t29+t30-t40-t41-t42-t43-t44-a31.*t8.*t12.*t15.*1.5e1;
            t32 = dq32.*t31;
            t34 = a31.*t2.*t3.*t5.*2.0e1;
            t35 = a51.*t2.*t3.*t5.*2.0e1;
            t36 = a11.*t7.*t8.*t9.*4.0e1;
            t37 = a31.*t8.*t12.*t15.*3.0e1;
            t38 = a11.*t2.*t8.*t12.*6.0e1;
            t39 = a31.*t3.*t7.*t16.*1.2e2;
            t33 = dq1.*(t18+t19+t20+t21+t22+t23-t34-t35-t36-t37-t38-t39);
            t45 = t24+t25+t26+t27+t28+t29+t30-t40-t41-t42-t43-t44-a31.*t2.*t8.*t12.*3.0e1;
            t46 = t3.^2;
            t47 = t8.^2;
            t48 = a41.*t5.*t46.*5.0;
            t49 = a01.*t10.*t47.*5.0;
            t50 = a21.*t7.*t8.*t9.*t15.*1.0e1;
            t51 = a21.*t3.*t8.*t12.*3.0e1;
            t52 = a41.*t2.*t3.*t7.*t16.*2.0e1;
            t53 = t48+t49+t50+t51+t52-a11.*t10.*t47.*5.0-a51.*t5.*t46.*5.0-a31.*t3.*t8.*t12.*3.0e1-a11.*t2.*t7.*t8.*t9.*2.0e1-a31.*t2.*t3.*t7.*t16.*2.0e1;
            t54 = a42.*t2.*t3.*t5.*4.0e1;
            t55 = a02.*t7.*t8.*t9.*2.0e1;
            t56 = a22.*t7.*t8.*t9.*2.0e1;
            t57 = a22.*t8.*t12.*t15.*6.0e1;
            t58 = a22.*t3.*t7.*t16.*6.0e1;
            t59 = a42.*t3.*t7.*t16.*6.0e1;
            t60 = a42.*t2.*t3.*t5.*2.0e1;
            t61 = a02.*t7.*t8.*t9.*1.0e1;
            t62 = a22.*t7.*t8.*t9.*1.0e1;
            t63 = a22.*t8.*t12.*t15.*1.5e1;
            t64 = a22.*t2.*t8.*t12.*3.0e1;
            t65 = a22.*t3.*t7.*t16.*3.0e1;
            t66 = a42.*t3.*t7.*t16.*3.0e1;
            t76 = a32.*t2.*t3.*t5.*1.0e1;
            t77 = a52.*t2.*t3.*t5.*1.0e1;
            t78 = a12.*t7.*t8.*t9.*2.0e1;
            t79 = a12.*t2.*t8.*t12.*3.0e1;
            t80 = a32.*t3.*t7.*t16.*6.0e1;
            t67 = t60+t61+t62+t63+t64+t65+t66-t76-t77-t78-t79-t80-a32.*t8.*t12.*t15.*1.5e1;
            t68 = dq32.*t67;
            t70 = a32.*t2.*t3.*t5.*2.0e1;
            t71 = a52.*t2.*t3.*t5.*2.0e1;
            t72 = a12.*t7.*t8.*t9.*4.0e1;
            t73 = a32.*t8.*t12.*t15.*3.0e1;
            t74 = a12.*t2.*t8.*t12.*6.0e1;
            t75 = a32.*t3.*t7.*t16.*1.2e2;
            t69 = dq1.*(t54+t55+t56+t57+t58+t59-t70-t71-t72-t73-t74-t75);
            t81 = t60+t61+t62+t63+t64+t65+t66-t76-t77-t78-t79-t80-a32.*t2.*t8.*t12.*3.0e1;
            t82 = a42.*t5.*t46.*5.0;
            t83 = a02.*t10.*t47.*5.0;
            t84 = a22.*t7.*t8.*t9.*t15.*1.0e1;
            t85 = a22.*t3.*t8.*t12.*3.0e1;
            t86 = a42.*t2.*t3.*t7.*t16.*2.0e1;
            t87 = t82+t83+t84+t85+t86-a12.*t10.*t47.*5.0-a52.*t5.*t46.*5.0-a32.*t3.*t8.*t12.*3.0e1-a12.*t2.*t7.*t8.*t9.*2.0e1-a32.*t2.*t3.*t7.*t16.*2.0e1;
            t88 = a43.*t2.*t3.*t5.*4.0e1;
            t89 = a03.*t7.*t8.*t9.*2.0e1;
            t90 = a23.*t7.*t8.*t9.*2.0e1;
            t91 = a23.*t8.*t12.*t15.*6.0e1;
            t92 = a23.*t3.*t7.*t16.*6.0e1;
            t93 = a43.*t3.*t7.*t16.*6.0e1;
            t94 = a43.*t2.*t3.*t5.*2.0e1;
            t95 = a03.*t7.*t8.*t9.*1.0e1;
            t96 = a23.*t7.*t8.*t9.*1.0e1;
            t97 = a23.*t8.*t12.*t15.*1.5e1;
            t98 = a23.*t2.*t8.*t12.*3.0e1;
            t99 = a23.*t3.*t7.*t16.*3.0e1;
            t100 = a43.*t3.*t7.*t16.*3.0e1;
            t110 = a33.*t2.*t3.*t5.*1.0e1;
            t111 = a53.*t2.*t3.*t5.*1.0e1;
            t112 = a13.*t7.*t8.*t9.*2.0e1;
            t113 = a13.*t2.*t8.*t12.*3.0e1;
            t114 = a33.*t3.*t7.*t16.*6.0e1;
            t101 = t94+t95+t96+t97+t98+t99+t100-t110-t111-t112-t113-t114-a33.*t8.*t12.*t15.*1.5e1;
            t102 = dq32.*t101;
            t104 = a33.*t2.*t3.*t5.*2.0e1;
            t105 = a53.*t2.*t3.*t5.*2.0e1;
            t106 = a13.*t7.*t8.*t9.*4.0e1;
            t107 = a33.*t8.*t12.*t15.*3.0e1;
            t108 = a13.*t2.*t8.*t12.*6.0e1;
            t109 = a33.*t3.*t7.*t16.*1.2e2;
            t103 = dq1.*(t88+t89+t90+t91+t92+t93-t104-t105-t106-t107-t108-t109);
            t115 = t94+t95+t96+t97+t98+t99+t100-t110-t111-t112-t113-t114-a33.*t2.*t8.*t12.*3.0e1;
            t116 = a43.*t5.*t46.*5.0;
            t117 = a03.*t10.*t47.*5.0;
            t118 = a23.*t7.*t8.*t9.*t15.*1.0e1;
            t119 = a23.*t3.*t8.*t12.*3.0e1;
            t120 = a43.*t2.*t3.*t7.*t16.*2.0e1;
            t121 = t116+t117+t118+t119+t120-a13.*t10.*t47.*5.0-a53.*t5.*t46.*5.0-a33.*t3.*t8.*t12.*3.0e1-a13.*t2.*t7.*t8.*t9.*2.0e1-a33.*t2.*t3.*t7.*t16.*2.0e1;
            t122 = a44.*t2.*t3.*t5.*4.0e1;
            t123 = a04.*t7.*t8.*t9.*2.0e1;
            t124 = a24.*t7.*t8.*t9.*2.0e1;
            t125 = a24.*t8.*t12.*t15.*6.0e1;
            t126 = a24.*t3.*t7.*t16.*6.0e1;
            t127 = a44.*t3.*t7.*t16.*6.0e1;
            t128 = a44.*t2.*t3.*t5.*2.0e1;
            t129 = a04.*t7.*t8.*t9.*1.0e1;
            t130 = a24.*t7.*t8.*t9.*1.0e1;
            t131 = a24.*t8.*t12.*t15.*1.5e1;
            t132 = a24.*t2.*t8.*t12.*3.0e1;
            t133 = a24.*t3.*t7.*t16.*3.0e1;
            t134 = a44.*t3.*t7.*t16.*3.0e1;
            t144 = a34.*t2.*t3.*t5.*1.0e1;
            t145 = a54.*t2.*t3.*t5.*1.0e1;
            t146 = a14.*t7.*t8.*t9.*2.0e1;
            t147 = a14.*t2.*t8.*t12.*3.0e1;
            t148 = a34.*t3.*t7.*t16.*6.0e1;
            t135 = t128+t129+t130+t131+t132+t133+t134-t144-t145-t146-t147-t148-a34.*t8.*t12.*t15.*1.5e1;
            t136 = dq32.*t135;
            t138 = a34.*t2.*t3.*t5.*2.0e1;
            t139 = a54.*t2.*t3.*t5.*2.0e1;
            t140 = a14.*t7.*t8.*t9.*4.0e1;
            t141 = a34.*t8.*t12.*t15.*3.0e1;
            t142 = a14.*t2.*t8.*t12.*6.0e1;
            t143 = a34.*t3.*t7.*t16.*1.2e2;
            t137 = dq1.*(t122+t123+t124+t125+t126+t127-t138-t139-t140-t141-t142-t143);
            t149 = t128+t129+t130+t131+t132+t133+t134-t144-t145-t146-t147-t148-a34.*t2.*t8.*t12.*3.0e1;
            t150 = a44.*t5.*t46.*5.0;
            t151 = a04.*t10.*t47.*5.0;
            t152 = a24.*t7.*t8.*t9.*t15.*1.0e1;
            t153 = a24.*t3.*t8.*t12.*3.0e1;
            t154 = a44.*t2.*t3.*t7.*t16.*2.0e1;
            t155 = t150+t151+t152+t153+t154-a14.*t10.*t47.*5.0-a54.*t5.*t46.*5.0-a34.*t3.*t8.*t12.*3.0e1-a14.*t2.*t7.*t8.*t9.*2.0e1-a34.*t2.*t3.*t7.*t16.*2.0e1;
            L2ydq_d = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t32+t33+dq31.*(t18+t19+t20+t21+t22+t23-a11.*t2.*t8.*t12.*6.0e1-a11.*t7.*t8.*t9.*4.0e1-a31.*t2.*t3.*t5.*2.0e1-a31.*t3.*t7.*t16.*1.2e2-a51.*t2.*t3.*t5.*2.0e1-a31.*t8.*t12.*t15.*3.0e1),t68+t69+dq31.*(t54+t55+t56+t57+t58+t59-a12.*t2.*t8.*t12.*6.0e1-a12.*t7.*t8.*t9.*4.0e1-a32.*t2.*t3.*t5.*2.0e1-a32.*t3.*t7.*t16.*1.2e2-a52.*t2.*t3.*t5.*2.0e1-a32.*t8.*t12.*t15.*3.0e1),t102+t103+dq31.*(t88+t89+t90+t91+t92+t93-a13.*t2.*t8.*t12.*6.0e1-a13.*t7.*t8.*t9.*4.0e1-a33.*t2.*t3.*t5.*2.0e1-a33.*t3.*t7.*t16.*1.2e2-a53.*t2.*t3.*t5.*2.0e1-a33.*t8.*t12.*t15.*3.0e1),t136+t137+dq31.*(t122+t123+t124+t125+t126+t127-a14.*t2.*t8.*t12.*6.0e1-a14.*t7.*t8.*t9.*4.0e1-a34.*t2.*t3.*t5.*2.0e1-a34.*t3.*t7.*t16.*1.2e2-a54.*t2.*t3.*t5.*2.0e1-a34.*t8.*t12.*t15.*3.0e1),t32+t33+dq31.*(t18+t19+t20+t21+t22+t23-t34-t35-t36-t37-t38-t39),t68+t69+dq31.*(t54+t55+t56+t57+t58+t59-t70-t71-t72-t73-t74-t75),t102+t103+dq31.*(t88+t89+t90+t91+t92+t93-t104-t105-t106-t107-t108-t109),t136+t137+dq31.*(t122+t123+t124+t125+t126+t127-t138-t139-t140-t141-t142-t143),dq1.*t45+dq31.*t45+dq32.*(t28+a01.*t7.*t8.*t9.*5.0-a11.*t2.*t8.*t12.*1.5e1-a11.*t7.*t8.*t9.*1.0e1-a31.*t2.*t3.*t5.*5.0+a21.*t7.*t8.*t9.*5.0+a21.*t3.*t7.*t16.*1.5e1+a41.*t2.*t3.*t5.*1.0e1-a31.*t2.*t8.*t12.*1.5e1-a31.*t3.*t7.*t16.*3.0e1-a51.*t2.*t3.*t5.*5.0+a41.*t3.*t7.*t16.*1.5e1),dq1.*t81+dq31.*t81+dq32.*(t64+a02.*t7.*t8.*t9.*5.0-a12.*t2.*t8.*t12.*1.5e1-a12.*t7.*t8.*t9.*1.0e1-a32.*t2.*t3.*t5.*5.0+a22.*t7.*t8.*t9.*5.0+a22.*t3.*t7.*t16.*1.5e1+a42.*t2.*t3.*t5.*1.0e1-a32.*t2.*t8.*t12.*1.5e1-a32.*t3.*t7.*t16.*3.0e1-a52.*t2.*t3.*t5.*5.0+a42.*t3.*t7.*t16.*1.5e1),dq1.*t115+dq31.*t115+dq32.*(t98+a03.*t7.*t8.*t9.*5.0-a13.*t2.*t8.*t12.*1.5e1-a13.*t7.*t8.*t9.*1.0e1-a33.*t2.*t3.*t5.*5.0+a23.*t7.*t8.*t9.*5.0+a23.*t3.*t7.*t16.*1.5e1+a43.*t2.*t3.*t5.*1.0e1-a33.*t2.*t8.*t12.*1.5e1-a33.*t3.*t7.*t16.*3.0e1-a53.*t2.*t3.*t5.*5.0+a43.*t3.*t7.*t16.*1.5e1),dq1.*t149+dq31.*t149+dq32.*(t132+a04.*t7.*t8.*t9.*5.0-a14.*t2.*t8.*t12.*1.5e1-a14.*t7.*t8.*t9.*1.0e1-a34.*t2.*t3.*t5.*5.0+a24.*t7.*t8.*t9.*5.0+a24.*t3.*t7.*t16.*1.5e1+a44.*t2.*t3.*t5.*1.0e1-a34.*t2.*t8.*t12.*1.5e1-a34.*t3.*t7.*t16.*3.0e1-a54.*t2.*t3.*t5.*5.0+a44.*t3.*t7.*t16.*1.5e1),0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t53,t87,t121,t155,t53,t87,t121,t155,a01.*t10.*t47.*(5.0./2.0)-a11.*t10.*t47.*(5.0./2.0)+a41.*t5.*t46.*(5.0./2.0)-a51.*t5.*t46.*(5.0./2.0)+a21.*t3.*t8.*t12.*1.5e1-a31.*t3.*t8.*t12.*1.5e1-a11.*t2.*t7.*t8.*t9.*1.0e1+a21.*t2.*t7.*t8.*t9.*1.0e1-a31.*t2.*t3.*t7.*t16.*1.0e1+a41.*t2.*t3.*t7.*t16.*1.0e1,a02.*t10.*t47.*(5.0./2.0)-a12.*t10.*t47.*(5.0./2.0)+a42.*t5.*t46.*(5.0./2.0)-a52.*t5.*t46.*(5.0./2.0)+a22.*t3.*t8.*t12.*1.5e1-a32.*t3.*t8.*t12.*1.5e1-a12.*t2.*t7.*t8.*t9.*1.0e1+a22.*t2.*t7.*t8.*t9.*1.0e1-a32.*t2.*t3.*t7.*t16.*1.0e1+a42.*t2.*t3.*t7.*t16.*1.0e1,a03.*t10.*t47.*(5.0./2.0)-a13.*t10.*t47.*(5.0./2.0)+a43.*t5.*t46.*(5.0./2.0)-a53.*t5.*t46.*(5.0./2.0)+a23.*t3.*t8.*t12.*1.5e1-a33.*t3.*t8.*t12.*1.5e1-a13.*t2.*t7.*t8.*t9.*1.0e1+a23.*t2.*t7.*t8.*t9.*1.0e1-a33.*t2.*t3.*t7.*t16.*1.0e1+a43.*t2.*t3.*t7.*t16.*1.0e1,a04.*t10.*t47.*(5.0./2.0)-a14.*t10.*t47.*(5.0./2.0)+a44.*t5.*t46.*(5.0./2.0)-a54.*t5.*t46.*(5.0./2.0)+a24.*t3.*t8.*t12.*1.5e1-a34.*t3.*t8.*t12.*1.5e1-a14.*t2.*t7.*t8.*t9.*1.0e1+a24.*t2.*t7.*t8.*t9.*1.0e1-a34.*t2.*t3.*t7.*t16.*1.0e1+a44.*t2.*t3.*t7.*t16.*1.0e1,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0],[4,14]);
        end
        function L2ydq_a = L2ydq_a_gen(obj,in1,in2)
            %L2YDQ_A_GEN
            %    L2YDQ_A = L2YDQ_A_GEN(IN1,IN2)

            %    This function was generated by the Symbolic Math Toolbox version 8.0.
            %    11-Dec-2019 16:46:17
            L2ydq_a = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,14]);
        end
        function dq_pos = dq_pos_gen_v2(obj, x)
            n = 7;
            dq_pre = x(n+1:end)';
            q_pre = x(1:n)';
            scale = obj.params.scale;
            
            D = obj.D_gen_rel(q_pre);
            Jsw = J_LeftToe(q_pre);
            Jsw = Jsw([1,3],:);

            % Invertible matrix
            H = [D -Jsw'; Jsw zeros(size(Jsw,1))];

            % Linear system Hy=r
            r = [D*dq_pre; zeros(size(Jsw,1),1)];
            y = H\r;

            dq_pos = y(1:length(dq_pre));

        end
        function D = D_gen_rel(obj,q_rel)
            scale = obj.params.scale;
            torso_mass = obj.params.torso_add;

            T = [ 0   0   -1   -1   0  0  0;
                  0   0   -1   0   0  -1  0;
                  0   0   -1   -1   -1  0  0;
                  0   0   -1   0   0  -1  -1;
                  1   0   0   0   0  0  0;
                  0   1   0   0   0  0  0;
                  0   0   -1   0   0  0  0];

            q = T*q_rel + [2*pi*ones(4,1);zeros(3,1)];
            q31=q(1); q32=q(2); q41=q(3); q42=q(4); y=q(5); z=q(6); q1=q(7);

            % gravity
            g=9.81;

            % lengths
            L1=0.625;
            L3=0.4;
            L4=0.4;

            % mass
            % M1=20*scale;
            % M3=6.8*scale;
            % M4=3.2*scale;
            M1=12*scale + torso_mass;
            M3=6.8*scale;
            M4=3.2*scale;

            % com
            MY1=.2;
            MZ1=4.;
            MZ3=1.11;
            MZ4=.41;

            % link inertia
            % XX1=2.22*scale;
            % XX3=.25*scale;
            % XX4=.10*scale;

            % XX1=1.33*scale;
            % XX3=.47*scale;
            % XX4=.2*scale;

            XX1=2.22*scale + 2.22*torso_mass/12;
            XX3=1.08*scale;
            XX4=0.93*scale;

            % D matrix
            D=zeros(7);
            D(1,1)=XX3+M4*L3^2;
            D(1,3)=MZ4*L3*cos(-q41+q31);
            D(1,5)=-cos(q31)*(M4*L3+MZ3);
            D(1,6)=-sin(q31)*(M4*L3+MZ3);
            D(2,2)=XX3+M4*L3^2;
            D(2,4)=MZ4*L3*cos(-q42+q32);
            D(2,5)=-cos(q32)*(M4*L3+MZ3);
            D(2,6)=-sin(q32)*(M4*L3+MZ3);
            D(3,1)=MZ4*L3*cos(-q41+q31);
            D(3,3)=XX4;
            D(3,5)=-MZ4*cos(q41);
            D(3,6)=-MZ4*sin(q41);
            D(4,2)=MZ4*L3*cos(-q42+q32);
            D(4,4)=XX4;
            D(4,5)=-MZ4*cos(q42);
            D(4,6)=-MZ4*sin(q42);
            D(5,1)=-cos(q31)*(M4*L3+MZ3);
            D(5,2)=-cos(q32)*(M4*L3+MZ3);
            D(5,3)=-MZ4*cos(q41);
            D(5,4)=-MZ4*cos(q42);
            D(5,5)=2*M3+2*M4+M1;
            D(5,7)=-cos(q1)*MZ1-sin(q1)*MY1;
            D(6,1)=-sin(q31)*(M4*L3+MZ3);
            D(6,2)=-sin(q32)*(M4*L3+MZ3);
            D(6,3)=-MZ4*sin(q41);
            D(6,4)=-MZ4*sin(q42);
            D(6,6)=2*M3+2*M4+M1;
            D(6,7)=-sin(q1)*MZ1+cos(q1)*MY1;
            D(7,5)=-cos(q1)*MZ1-sin(q1)*MY1;
            D(7,6)=-sin(q1)*MZ1+cos(q1)*MY1;
            D(7,7)=XX1;

            D = T'*D*T;
        end
        function C = C_gen_rel(obj, q_rel,dq_rel)
            scale = obj.params.scale;
            torso_mass = obj.params.torso_add;

            T = [ 0   0   -1   -1   0  0  0;
                  0   0   -1   0   0  -1  0;
                  0   0   -1   -1   -1  0  0;
                  0   0   -1   0   0  -1  -1;
                  1   0   0   0   0  0  0;
                  0   1   0   0   0  0  0;
                  0   0   -1   0   0  0  0];

            q = T*q_rel + [2*pi*ones(4,1);zeros(3,1)];
            dq = T*dq_rel;
            q31=q(1); q32=q(2); q41=q(3); q42=q(4); y=q(5); z=q(6); q1=q(7);
            dq31=dq(1); dq32=dq(2); dq41=dq(3); dq42=dq(4); dy=dq(5); dz=dq(6); dq1=dq(7);

            % gravity
            g=9.81;

            % lengths
            L1=0.625;
            L3=0.4;
            L4=0.4;

            % mass
            % M1=20*scale;
            % M3=6.8*scale;
            % M4=3.2*scale;
            M1=12*scale + torso_mass;
            M3=6.8*scale;
            M4=3.2*scale;

            % com
            MY1=.2;
            MZ1=4.;
            MZ3=1.11;
            MZ4=.41;

            % link inertia
            % XX1=2.22*scale;
            % XX3=.25*scale;
            % XX4=.10*scale;

            % XX1=1.33*scale;
            % XX3=.47*scale;
            % XX4=.2*scale;

            XX1=2.22*scale + 2.22*torso_mass/12;
            XX3=1.08*scale;
            XX4=0.93*scale;

            % C matrix
            C=zeros(7);
            C(1,3)=MZ4*L3*sin(-q41+q31)*dq41;
            C(2,4)=MZ4*L3*sin(-q42+q32)*dq42;
            C(3,1)=-MZ4*L3*sin(-q41+q31)*dq31;
            C(4,2)=-MZ4*L3*sin(-q42+q32)*dq32;
            C(5,1)=sin(q31)*(M4*L3+MZ3)*dq31;
            C(5,2)=sin(q32)*(M4*L3+MZ3)*dq32;
            C(5,3)=MZ4*sin(q41)*dq41;
            C(5,4)=MZ4*sin(q42)*dq42;
            C(5,7)=(sin(q1)*MZ1-cos(q1)*MY1)*dq1;
            C(6,1)=-cos(q31)*(M4*L3+MZ3)*dq31;
            C(6,2)=-cos(q32)*(M4*L3+MZ3)*dq32;
            C(6,3)=-MZ4*cos(q41)*dq41;
            C(6,4)=-MZ4*cos(q42)*dq42;
            C(6,7)=(-cos(q1)*MZ1-sin(q1)*MY1)*dq1;

            C = T'*C*T;
        end
        function G = G_gen_rel(obj, q_rel)
            scale = obj.params.scale;
            torso_mass = obj.params.torso_add;
            T = [ 0   0   -1   -1   0  0  0;
                  0   0   -1   0   0  -1  0;
                  0   0   -1   -1   -1  0  0;
                  0   0   -1   0   0  -1  -1;
                  1   0   0   0   0  0  0;
                  0   1   0   0   0  0  0;
                  0   0   -1   0   0  0  0];

            q = T*q_rel + [2*pi*ones(4,1);zeros(3,1)];
            q31=q(1); q32=q(2); q41=q(3); q42=q(4); y=q(5); z=q(6); q1=q(7);

            % gravity
            g=9.81;

            % lengths
            L1=0.625;
            L3=0.4;
            L4=0.4;

            % mass
            % M1=20*scale;
            % M3=6.8*scale;
            % M4=3.2*scale;
            M1=12*scale + torso_mass;
            M3=6.8*scale;
            M4=3.2*scale;

            % com
            MY1=.2;
            MZ1=4.;
            MZ3=1.11;
            MZ4=.41;

            % link inertia
            % XX1=2.22*scale;
            % XX3=.25*scale;
            % XX4=.10*scale;

            % XX1=1.33*scale;
            % XX3=.47*scale;
            % XX4=.2*scale;

            XX1=2.22*scale + 2.22*torso_mass/12;
            XX3=1.08*scale;
            XX4=0.93*scale;

            % G matrix
            G=zeros(7,1);
            G(1)=-g*sin(q31)*(MZ3+L3*M4);
            G(2)=-g*sin(q32)*(MZ3+L3*M4);
            G(3)=-g*MZ4*sin(q41);
            G(4)=-g*MZ4*sin(q42);
            G(6)=g*(M1+2*M3+2*M4);
            G(7)=-g*(sin(q1)*MZ1-cos(q1)*MY1);

            G = T'*G;
        end
%         function y_max_exceed_ = y_max_exceed(obj, x)
%         end
%         
%         function lf_y_max_exceed_ = lf_y_max_exceed(obj, x)
%         end
%         
%         function lg_y_max_exceed_ = lg_y_max_exceed(obj, x)
%         end
%         
%         function lglf_y_max_exceed_ = lglf_y_max_exceed(obj, x)
%         end
%         
%         function l2f_y_max_exceed_ = l2f_y_max_exceed(obj, x)
%         end
%         
%         function y_min_exceed_ = y_min_exceed(obj, x)
%         end
%         
%         function lf_y_min_exceed_ = lf_y_min_exceed(obj, x)
%         end
%         
%         function lg_y_min_exceed_ = lg_y_min_exceed(obj, x)
%         end
%         
%         function lglf_y_min_exceed_ = lglf_y_min_exceed(obj, x)
%         end
%         
%         function l2f_y_min_exceed_ = l2f_y_min_exceed(obj, x)
%         end
    end
end

