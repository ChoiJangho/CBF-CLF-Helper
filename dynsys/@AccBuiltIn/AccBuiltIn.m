%% Main Reference
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

classdef AccBuiltIn < CtrlAffineSys    
    methods
        function obj = AccBuiltIn(params)
            % Always using built-in option for setup.
            obj = obj@CtrlAffineSys(params, 'built-in');            
        end
        function Fr = getFr(obj, x)
            v = x(2, :);
            Fr = obj.params.f0 + obj.params.f1 * v + obj.params.f2 * v.^2;
        end
        function f_ = f(obj, x)
            v = x(2, :);
            Fr = getFr(obj, x);
            f_ = [v; -Fr/obj.params.m; obj.params.v0-v];
        end
        function g_ = g(obj, x)
            g_ = [0; 1/obj.params.m; 0];
        end
        function V = clf(obj, x)
            v = x(2, :);
            V = (v - obj.params.vd).^2;
        end
        function LfV = lf_clf(obj, x)
            v = x(2, :);
            Fr = getFr(obj, x);
            LfV = - 2 * Fr .* (v - obj.params.vd) / obj.params.m;
        end
        function LgV = lg_clf(obj, x)
            v = x(2, :);
            LgV = 2 * (v - obj.params.vd) / obj.params.m;
        end
        function B = cbf(obj, x)
            v = x(2);
            z = x(3);
            T = obj.params.T;
            cd = obj.params.cd;
            v0 = obj.params.v0;
            g = obj.params.g;
            B = z - T * v - 0.5 * (v0 - v).^2 / (cd * g);
        end
        function LfB = lf_cbf(obj, x)
            v = x(2);
            T = obj.params.T;
            cd = obj.params.cd;
            v0 = obj.params.v0;
            g = obj.params.g;
            m = obj.params.m;
            Fr = getFr(obj, x);
            LfB = (T + (v - v0) / (cd * g)) .* Fr / m + (v0 - v);            
        end
        function LgB = lg_cbf(obj, x)
            v = x(2);
            T = obj.params.T;
            cd = obj.params.cd;
            v0 = obj.params.v0;
            g = obj.params.g;
            m = obj.params.m;
            LgB = -(T + (v - v0) / (cd * g)) / m;
        end
    end
end
