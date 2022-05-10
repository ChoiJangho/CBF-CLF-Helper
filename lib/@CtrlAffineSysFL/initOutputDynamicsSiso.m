function initOutputDynamicsSiso(obj, symbolic_x, symbolic_f, symbolic_g, ...
    symbolic_y, symbolic_z)
% Construct essential property of IO-FL, prepare for CLF
%   ex) F, G, F_epsilon, P, Q, A, gamma
% From 
%   params => epsilon_FL, Q_F, Kd_FL, Kp_FL
% To
%   object => ydim, udim, y, rel_deg_y, lf_y, lg_y, l2f_y, lglf_y
%          => y_exceed_max, lf_y_exceed_max, lg_y_exceed_max,
%          l2f_y_exceed_max, lglf_y_exeed_max as a function
%          => F_FL, G_FL, Gram_CLF_FL(P), F_FL_eps, clf_rate
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
    
    if length(symbolic_y) > 1
        error("SISO Not supported for high-dimensional y. %s", ...
            "Consider using output_option 'mimo' or 'phase'");
    end
    if obj.udim > 1
        error("SISO Not supported for high-dimensional u. %s", ...
            "Consider using output_option 'mimo' or 'phase'");
    end
    obj.ydim = 1;

    %% Set up and save function handles of output and its lie derivatives.
    if ~isfield(obj.params, 'rel_deg_y')
        error("Currently auto relative degree is not supported. %s", ...
            "please specify it with params.rel_deg_y.");
    end
    obj.y_sym = matlabFunction(symbolic_y, 'vars', {x});

    %% Evaluating high-order derivatives of y and the jordan form.
    obj.rel_deg_y = obj.params.rel_deg_y;
    obj.lfs_y_sym = cell(obj.rel_deg_y, 1);
    
    if obj.rel_deg_y == 1
        dy = simplify(jacobian(symbolic_y, symbolic_x));
        lf_y_ = simplify(dy * f_);
        lg_y_ = simplify(dy * g_);
        obj.lfs_y_sym{1} = matlabFunction(lf_y_, 'vars', {x});
        obj.lglfr_y_sym = matlabFunction(lg_y_, 'vars', {x});
        obj.xi_sym = obj.y_sym;
    else
        xi_ = symbolic_y;
        dy = simplify(jacobian(symbolic_y, symbolic_x));
        lf_y_next = simplify(dy * f_);
        for i = 1:obj.rel_deg_y-1
            xi_ = [xi_; lf_y_next];
            obj.lfs_y_sym{i} = matlabFunction(lf_y_next, 'vars', {x});
            y_deriv_next = simplify(jacobian(lf_y_next, symbolic_x));
            lf_y_next = simplify(y_deriv_next * f_);
        end
        obj.lfs_y_sym{obj.rel_deg_y} = matlabFunction(lf_y_next, 'vars', {x});
        lglfr_y = simplify(y_deriv_next * g_);
        obj.lglfr_y_sym = matlabFunction(lglfr_y, 'vars', {x});
        obj.xi_sym = matlabFunction(xi_, 'vars', {x});
    end
    
    if size(symbolic_z, 1) ~= obj.xdim - obj.rel_deg_y
        error("dimension of the zero dynamics coordinate is not xdim - rel_deg_y");
    end
    if size(symbolic_z, 1) ~=0
            obj.z_sym = matlabFunction(symbolic_z, 'vars', {x});

        dz = simplify(jacobian(symbolic_z, symbolic_x));
        lf_z_ = simplify(dz * f_);
        lg_z_ = simplify(dz * g_);
        if ~isAlways(norm(lg_z_) == 0)
            disp(["WARNING: Zero dynamics coordinate might not be well defined, ", ...
                "detected internal dynamics dependence on u."]);
        end
        obj.internal_dynamics_sym = matlabFunction(lf_z_, 'vars', {x});
    end
    
    if isfield(obj.params, 'K_siso')
        if length(obj.params.K_siso) ~= obj.rel_deg_y
            error("Wrong obj.K_siso size.");
        end
        obj.K_siso = obj.params.K_siso;
    end
end
