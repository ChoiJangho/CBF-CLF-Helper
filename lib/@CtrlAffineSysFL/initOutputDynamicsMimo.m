function initOutputDynamicsMimo(obj, symbolic_x, symbolic_f, symbolic_g, ...
    symbolic_y, symbolic_z)

    if isempty(symbolic_x) || isempty(symbolic_f) || isempty(symbolic_g)
        error('x, f, g is empty. Create a class function defineSystem and define your dynamics with symbolic expression.');
    end
    
    if ~isa(symbolic_f, 'sym')
        f_ = sym(symbolic_f);
    else
        f_ = symbolic_f;
    end
    if ~isa(symbolic_g, 'sym')
        g_ = sym(symbolic_g);
    else
        g_ = symbolic_g;
    end
    x = symbolic_x;
    
    if length(symbolic_y) ~= obj.udim
        error("length of symbolic_y must be same with obj.udim.");
    end
    obj.ydim = length(symbolic_y);

    %% Set up and save function handles of output and its lie derivatives.
    if ~isfield(obj.params, 'rel_deg_y')
        error("Currently auto relative degree is not supported. %s", ...
            "specify it with params.rel_deg_y.");
    end
    obj.y_sym = matlabFunction(symbolic_y, 'vars', {x});

    %% Evaluating high-order derivatives of y and the jordan form.
    obj.rel_deg_y = obj.params.rel_deg_y;
    if length(obj.rel_deg_y) ~= obj.ydim
        error("Wrong params.rel_deg_y size. It should be (obj.ydim, 1) cell.");
    end
    obj.lfs_y_sym = cell(obj.ydim, 1);
    obj.lglfr_y_sym = cell(obj.ydim, 1);
    obj.xi_sym = cell(obj.ydim, 1);
    decoupling_matrix_ = [];
    for i = 1:obj.ydim
        rel_deg_i = obj.rel_deg_y(i);
        obj.lfs_y_sym{i} = cell(rel_deg_i, 1);
        % Iteratively deriving high-order derivatives of y_i.
        lf_y_next = symbolic_y(i);
        xi_i = lf_y_next;        
        for j = 1:rel_deg_i-1
            y_deriv_next = simplify(jacobian(lf_y_next, x));
            lf_y_next = simplify(y_deriv_next * f_);
            xi_i = [xi_i; lf_y_next];        
            obj.lfs_y_sym{i}{j} = matlabFunction(lf_y_next, 'vars', {x});
        end
        % Highest order derivative of y_i.
        y_deriv_next = simplify(jacobian(lf_y_next, x));
        lf_y_next = simplify(y_deriv_next * f_);
        obj.lfs_y_sym{i}{rel_deg_i} = matlabFunction(lf_y_next, 'vars', {x});
        lglfr_y_i = simplify(y_deriv_next * g_);
        decoupling_matrix_ = [decoupling_matrix_; lglfr_y_i];
        obj.lglfr_y_sym{i} = matlabFunction(lglfr_y_i, 'vars', {x});
        obj.xi_sym{i} = matlabFunction(xi_i, 'vars', {x});
    end
    obj.decoupling_matrix_sym = matlabFunction(decoupling_matrix_, 'vars', {x});
    
    if length(symbolic_z) ~= obj.xdim - sum(obj.rel_deg_y)
        error("dimension of the zero dynamics coordinate is not xdim - rel_deg_y");
    end
    obj.zdim = length(symbolic_z);
    if obj.zdim ~=0
        obj.z_sym = matlabFunction(symbolic_z, 'vars', {x}); 
        obj.lf_z_sym = cell(obj.zdim, 1);
        obj.lg_z_sym = cell(obj.zdim, 1);        
        for i = 1:obj.zdim
            dz_i = simplify(jacobian(symbolic_z(i), x));
            lf_z_i = simplify(dz_i * f_);
            lg_z_i = simplify(dz_i * g_);
            obj.lf_z_sym{i} = matlabFunction(lf_z_i, 'vars', {x});
            obj.lg_z_sym{i} = matlabFunction(lg_z_i, 'vars', {x});
        end        
    end
    
    if isfield(obj.params, 'K_mimo')
        if length(obj.params.K_mimo) ~= obj.ydim
            error("Wrong obj.K_mimo size.");
        end
        obj.K_mimo = obj.params.K_mimo;
    end
end
