%% Main reference: Student Workbook - Linear Pendulum Gantry Experiment, Quanser

classdef QuanserCartPoleInputDelay < QuanserCartPole
    methods
        function obj = QuanserCartPoleInputDelay(params, IP02_LOAD_TYPE, PEND_TYPE)
            if nargin < 2
                IP02_LOAD_TYPE = 'NO_LOAD';
            end
            if nargin < 3
                PEND_TYPE = 'LONG_24IN';
            end                
            obj = obj@QuanserCartPole(params, IP02_LOAD_TYPE, PEND_TYPE);
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
            clf = x(1:4)' * P * x(1:4);
        end
    end
end
