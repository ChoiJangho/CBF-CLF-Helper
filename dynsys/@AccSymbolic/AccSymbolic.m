%% Main Reference
% Aaron Ames et al. Control Barrier Function based Quadratic Programs 
% with Application to Adaptive Cruise Control, CDC 2014, Table 1.

classdef AccSymbolic < CtrlAffineSys    
    methods        
        function obj = AccSymbolic(params, varargin)
            % Always using symbolic option for setup.
            obj = obj@CtrlAffineSys(params, 'symbolic', varargin{:});
        end
        function Fr = getFr(obj, x)
            v = x(2);
            Fr = obj.params.f0 + obj.params.f1 * v + obj.params.f2 * v^2;
        end
    end
end
