classdef Car4DGridCbf < Car4D & CtrlAffineSysGridCbf
    methods
        function obj = Car4DGridCbf(params)
            obj@Car4D(params);
            obj@CtrlAffineSysGridCbf(params);
        end
    end
    
    methods(Static)
        function data0 = get_target_function(grid, params)
            xo = params.xo; 
            yo = params.yo;
            Ro = params.Ro;
            max_acc = params.u_max(2);
            
            data0 = zeros(grid.N');
            grid_xy = createGrid(grid.min(1:2), grid.max(1:2), grid.N(1:2));

            for i = 1:length(grid.vs{3})
                for j = 1:length(grid.vs{4})
                    theta = grid.vs{3}(i);
                    v = grid.vs{4}(j);
                    if params.apply_cbf_smooth_margin
                        smooth_margin = (1 - sqrt(1 - (1 - 2 * min(max(v, 0), params.v_max) / params.v_max)^2));
                    else                
                        smooth_margin = 0;
                    end
                    stopping_distance = 0.5 * v^2 / max_acc;
                    avoid_center = [-0.5 * stopping_distance * cos(theta) + xo;
                        -0.5 * stopping_distance * sin(theta) + yo];
                    avoid_radius = Ro + stopping_distance * 0.5 + smooth_margin;
                    data0_2d = shapeCylinder(grid_xy, [], avoid_center, avoid_radius);
                    data0(:, :, i, j) = data0_2d;
                end
            end
        end
    end
end