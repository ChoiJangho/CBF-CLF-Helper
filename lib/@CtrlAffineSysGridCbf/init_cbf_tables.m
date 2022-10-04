function init_cbf_tables(obj, grid, cbf_table)
    obj.grid = grid;
    obj.cbf_grid = cbf_table;
    obj.dcbf_grid = computeGradients(grid, cbf_table);
    
    %% evaluate the vector fields on the tables.
    cell_size = size(obj.grid.xs{1});
    cell_dim = length(cell_size);
    x = reshape(obj.grid.xs, [ones(1, cell_dim), obj.xdim]);
    x = cell2mat(x);
    x = permute(x, [cell_dim+1, 1:cell_dim]);
    x = reshape(x, obj.xdim, []);
    fs = obj.f(x);
    obj.f_grid = cell(obj.xdim, 1);
    for i = 1:obj.xdim
        obj.f_grid{i} = reshape(fs(i, :), cell_size);        
    end    
    gs = obj.g(x);
    obj.g_grid = cell(obj.xdim, obj.udim);
    for i = 1:obj.xdim
        for j = 1:obj.udim
            obj.g_grid{i, j} = reshape(gs(i, j, :), cell_size);
        end
    end
    
    %% evaluate the lie derivatives
    lf_cbf = 0;
    lg_cbf = cell(1, obj.udim);
    lg_cbf(:) = {0};
    for i = 1:obj.xdim
        lf_cbf = lf_cbf + obj.dcbf_grid{i} .* obj.f_grid{i};
        for j = 1:obj.udim
            lg_cbf{j} = lg_cbf{j} + obj.dcbf_grid{i} .* obj.g_grid{i, j}; 
        end
    end
    obj.lf_cbf_grid = lf_cbf;
    obj.lg_cbf_grid = lg_cbf;
end