classdef CtrlAffineSysGridCbf < CtrlAffineSys
    properties
        grid
        cbf_grid
        dcbf_grid
        f_grid
        g_grid
        lf_cbf_grid
        lg_cbf_grid
    end
    
    methods
        function obj = CtrlAffineSysGridCbf(params, varargin)
            obj@CtrlAffineSys(params, varargin{:});
        end
        
        function Bs = cbf(obj, x)
            Bs = obj.eval_value(obj.grid, obj.cbf_grid, x);
        end
        
        function LfBs = lf_cbf(obj, x)
            LfBs = obj.eval_value(obj.grid, obj.lf_cbf_grid, x);
        end
        
        function LgBs = lg_cbf(obj, x)
            LgBs = obj.eval_value(obj.grid, obj.lg_cbf_grid, x);
        end        
    end
            
    
    methods(Static)
        vs = eval_value(grid, table, xs, interp_method)
    end
end