function init_sys(obj, params)
%% Functions that initilaize dynamic system
% Built in
    % implant the params' parameter to object's parameters
    % ex) x_dim, u_dim, n_clf, n_cbf, clf_rate, cbf_rate ...
    if obj.is_sys_initialized
        return
    end
    obj.params = params;
    %% Symbolic System
    if strcmp(obj.setup_option, 'symbolic') || strcmp(obj.setup_option, 'symbolic-built-in')
        disp(['Setting up the dynamics, CLFs, CBFs from defined symbolic expressions.', ...
            '(This might take time.)']);
        generate_clone = false;
        if strcmp(obj.setup_option, 'symbolic-built-in')
            generate_clone = true;
            original_class_name = class(obj);
            clone_class_name = strcat(original_class_name, 'Clone');
            clone_class_path = fileparts(fileparts(which(class(obj))));
            status = mkdir(clone_class_path, strcat('@', clone_class_name));
            clone_class_path = fullfile(clone_class_path, strcat('@', clone_class_name));
            %% TODO (Unfinished)
            disp(['Built-in class clone will be created at', clone_class_path]);
            syms obj_
        end
        
        % get symbolic expressions for x, f, g, clf, cbf.
        [x, f_, g_] = obj.defineSystem(params);
        assume(x, 'real');
        if isempty(x) || isempty(f_) || isempty(g_)
            error("x, f, g is empty. Create a class function defineSystem %s", ...
            "and define your dynamics with symbolic expression.");
        end
        if ~isa(f_, 'sym')
            f_ = sym(f_);
        end
        if ~isa(g_, 'sym')
            g_ = sym(g_);
        end
        % Setting state and input dimension.
        obj.xdim = size(x, 1);
        obj.udim = size(g_, 2);
        
        obj.f_sym = matlabFunction(f_, 'vars', {x});
        obj.g_sym = matlabFunction(g_, 'vars', {x});
        if generate_clone
            f_file_str = fullfile(clone_class_path, 'f');
            matlabFunction(f_, 'file', f_file_str, 'vars', {obj_, x});
            g_file_str = fullfile(clone_class_path, 'g');
            matlabFunction(g_, 'file', g_file_str, 'vars', {obj_, x});
        end      

        % Retrieve all clfs and cbfs that is defined.
        clf_all = obj.defineClf(params, x);
        cbf_all = obj.defineCbf(params, x);
        
        clf_mask = get_mask('clf', params, length(clf_all));
        cbf_mask = get_mask('cbf', params, length(cbf_all));
        
        clf_ = clf_all(clf_mask);
        cbf_ = cbf_all(cbf_mask);

        if ~isempty(clf_)
            if ~iscell(clf_)
                if length(clf_) ~=1
                    error("Defined CBF should be a scalar. For multiple CBFs, %s", ...
                        "use a cell array.");
                end
                clf_ = {clf_};
            end
            
            obj.n_clf = length(clf_);
            clf_sym = cell(obj.n_clf, 1);
            lf_clf_sym = cell(obj.n_clf, 1);
            lg_clf_sym = cell(obj.n_clf, 1);
            syms('clf_sym_array', [obj.n_clf, 1]);
            syms('lf_clf_', [obj.n_clf, 1]);
            syms('lg_clf_', [obj.n_clf, 1]);
            
            
            for i_clf = 1:obj.n_clf
                clf_sym{i_clf} = matlabFunction(clf_{i_clf}, 'vars', {x});
                dclf_i = simplify(jacobian(clf_{i_clf}, x));
                lf_clf_i = dclf_i * f_;
                lg_clf_i = dclf_i * g_;
                
                lf_clf_sym{i_clf} = matlabFunction(lf_clf_i, 'vars', {x});
                if all(isAlways(simplify(lg_clf_i) == 0, 'Unknown', 'false'))
                    error('Relative degree of the defined %d-th CLF > 1. %s', ...
                    [i_clf, 'Currently, High-order relative degree is not supported.']);
                end
                lg_clf_sym{i_clf} = matlabFunction(lg_clf_i, 'vars', {x});
                
                if generate_clone
                    clf_sym_array(i_clf) = clf_{i_clf};
                    lf_clf_(i_clf) = lf_clf_i;
                    lg_clf_(i_clf) = lg_clf_i;
                end
            
            end
            obj.clf_sym = clf_sym;             
            obj.lf_clf_sym = lf_clf_sym;
            obj.lg_clf_sym = lg_clf_sym;
            if generate_clone
                clf_file_str = fullfile(clone_class_path, 'clf');
                matlabFunction(clf_sym_array, 'file', clf_file_str, 'vars', {obj_, x});
                lf_clf_file_str = fullfile(clone_class_path, 'lf_clf');
                matlabFunction(lf_clf_, 'file', lf_clf_file_str, 'vars', {obj_, x});
                %% TODO: make sure that this works when udim > 1.
                lg_clf_file_str = fullfile(clone_class_path, 'lg_clf');
                matlabFunction(lg_clf_, 'file', lg_clf_file_str, 'vars', {obj_, x});
            end
        else
            obj.n_clf = 0;
        end
        if ~isempty(cbf_)
            if ~iscell(cbf_)
                if length(cbf_) ~=1
                    error("Defined CBF should be a scalar. For multiple CBFs, %s", ...
                        "use a cell array.");
                end
                cbf_ = {cbf_};
            end
            obj.n_cbf = length(cbf_);
            cbf_sym = cell(obj.n_cbf, 1);
            lf_cbf_sym = cell(obj.n_cbf, 1);
            lg_cbf_sym = cell(obj.n_cbf, 1);
            syms('cbf_sym_array', [obj.n_cbf, 1]);
            syms('lf_cbf_', [obj.n_cbf, 1]);
            syms('lg_cbf_', [obj.n_cbf, 1]);
            
            for i_cbf = 1:obj.n_cbf
                cbf_sym{i_cbf} = matlabFunction(cbf_{i_cbf}, 'vars', {x});

                dcbf_i = simplify(jacobian(cbf_{i_cbf}, x));
                lf_cbf_i = dcbf_i * f_;
                lg_cbf_i = dcbf_i * g_;        
                lf_cbf_sym{i_cbf} = matlabFunction(lf_cbf_i, 'vars', {x});
                if all(isAlways(simplify(lg_cbf_i) == 0, 'Unknown', 'false'))
                    error('Relative degree of the defined %d-th CBF > 1. %s', ...
                    [i_cbf, 'Currently, High-order relative degree is not supported.']);
                end
                lg_cbf_sym{i_cbf} = matlabFunction(lg_cbf_i, 'vars', {x});
                
                if generate_clone
                    cbf_sym_array(i_cbf) = cbf_{i_cbf};
                    lf_cbf_(i_cbf) = lf_cbf_i;
                    lg_cbf_(i_cbf) = lg_cbf_i;
                end
            end
            obj.cbf_sym = cbf_sym;             
            obj.lf_cbf_sym = lf_cbf_sym;
            obj.lg_cbf_sym = lg_cbf_sym;
            if generate_clone
                cbf_file_str = fullfile(clone_class_path, 'cbf');
                matlabFunction(cbf_sym_array, 'file', cbf_file_str, 'vars', {obj_, x});
                lf_cbf_file_str = fullfile(clone_class_path, 'lf_cbf');
                matlabFunction(lf_cbf_, 'file', lf_cbf_file_str, 'vars', {obj_, x});
                %% TODO: make sure that this works when udim > 1.
                lg_cbf_file_str = fullfile(clone_class_path, 'lg_cbf');
                matlabFunction(lg_cbf_, 'file', lg_cbf_file_str, 'vars', {obj_, x});                
            end
        else
            obj.n_cbf = 0;
        end
        
    elseif strcmp(obj.setup_option, 'built-in')
        %% BuiltIn System
        % extract param information and inject to object's property
        % xdim, udim, n_clf, n_cbf 
        if ~isfield(params, 'xdim')
            error("xdim should be specified for built-in setup.");
        end
        obj.xdim = params.xdim;
        if ~isfield(params, 'udim')
            error("udim should be specified for built-in setup.");
        end
        obj.udim = params.udim;
        x_test = zeros(obj.xdim, 1);
        % Beta ver. active constraint
        n_clf_total = length(obj.clf_all(x_test));
        n_cbf_total = length(obj.cbf_all(x_test));
        
        clf_mask = get_mask('clf', params, n_clf_total);
        cbf_mask = get_mask('cbf', params, n_cbf_total);
        
        paren = @(x, varargin) x(varargin{:});
        obj.clf_builtin = @(x) paren(obj.clf_all(x), clf_mask);
        obj.lf_clf_builtin = @(x) paren(obj.lf_clf_all(x), clf_mask);
        obj.lg_clf_builtin = @(x) paren(obj.lg_clf_all(x), clf_mask);
        obj.cbf_builtin = @(x) paren(obj.cbf_all(x), cbf_mask);
        obj.lf_cbf_builtin = @(x) paren(obj.lf_cbf_all(x), cbf_mask);
        obj.lg_cbf_builtin = @(x) paren(obj.lg_cbf_all(x), cbf_mask);
        
        try
            clf_test = obj.clf(x_test);
            n_clf = length(clf_test);
        catch e
            n_clf = 0;
        end
        obj.n_clf = n_clf;
        
        try
            cbf_test = obj.cbf(x_test);
            n_cbf = length(cbf_test);
        catch e
            n_cbf = 0;
        end
        obj.n_cbf = n_cbf;               
    else
        error("Undefined setup_option.");
    end
    
    if isfield(params, 'dims_angle')
        obj.dims_angle = logical(params.dims_angle);
    end

    
    %% Parse parameters for both setup_option.
    % set cbf rate, clf rate
    if isfield(params, 'clf') && isfield(params.clf, 'rate')
        obj.clf_rate = params.clf.rate;
    elseif isfield(params, 'clf_rate')
        obj.clf_rate = params.clf_rate;
    elseif obj.n_clf >= 1
        error("params.clf.rate or params.clf_rate should be provided %s", ...
            "in order to use CLF.");
    end
    if isfield(params, 'cbf') && isfield(params.cbf, 'rate')
        obj.cbf_rate = params.cbf.rate;
    elseif isfield(params, 'cbf_rate')
        obj.cbf_rate = params.cbf_rate;
    elseif obj.n_cbf >= 1
        error("params.cbf.rate or params.cbf_rate should be provided %s", ...
            "in order to use CBF.");
    end
    if isfield(params, 'u_min')
        if length(params.u_min) == 1
            obj.u_min = params.u_min * ones(obj.udim, 1);
        elseif length(params.u_min) ~= obj.udim
            error("Invalid size of params.u_min.");
        else
            if isrow(params.u_min)
                obj.u_min = params.u_min';
            else
                obj.u_min = params.u_min;
            end
        end
    end
    if isfield(params, 'u_max')
        if length(params.u_max) == 1
            obj.u_max = params.u_max * ones(obj.udim, 1);
        elseif length(params.u_max) ~= obj.udim
            error("Invalid size of params.u_max.");
        else
            if isrow(params.u_max)
                obj.u_max = params.u_max';
            else
                obj.u_max = params.u_max;
            end
        end
    end
    %% Parse weight parameters.
    % obj.weight_input is saved as (udim x udim) matrix.
    if isfield(params, 'weight')
        if isfield(params.weight, 'input')
            weight_input = params.weight.input;
            if length(weight_input) == 1
                obj.weight_input = weight_input * eye(obj.udim);
            elseif isrow(weight_input) && length(weight_input) == obj.udim
                obj.weight_input = diag(weight_input);
            elseif iscolumn(params.weight.input)
                obj.weight_input = diag(weight_input);
            elseif all(size(obj.params.weight.input) == obj.udim)
                obj.weight_input = weight_input;
            else
                error("params.weight.input should be either a scalar value%s", ...
                    "(udim) vector that contains the diagonal elements of ", ...
                    "the weight matrix, or the (udim, udim) full matrix.")
            end
        end
    end
    if isfield(params, 'weight_input')
        weight_input = params.weight_input;
        if length(weight_input) == 1
            obj.weight_input = weight_input * eye(obj.udim);
        elseif isrow(weight_input) && length(weight_input) == obj.udim
            obj.weight_input = diag(weight_input);
        elseif iscolumn(params.weight.input)
            obj.weight_input = diag(weight_input);
        elseif all(size(obj.params.weight.input) == obj.udim)
            obj.weight_input = weight_input;
        else
            error("params.weight_input should be either a scalar value%s", ...
                "(udim) vector that contains the diagonal elements of ", ...
                "the weight matrix, or the (udim, udim) full matrix.")
        end
    end
    if isempty(obj.weight_input)
        obj.weight_input = eye(obj.udim);
    end
    % obj.weight_slack is saved as a scalar value.
    if isfield(params, 'weight')
        if isfield(params.weight, 'slack')
            weight_slack = params.weight.slack;
            if length(weight_slack) == 1
                obj.weight_slack = weight_slack;
            else
                error("Default slack weight should be a scalar value. %s", ...
                    "If you want to specify different slack weights for ", ...
                    "each constraint, pass it to the controllers as ", ...
                    "additional argument 'weight_slack'");
            end
        end
    end    
    if isfield(params, 'weight_slack')
        weight_slack = params.weight_slack;
        if length(weight_slack) == 1
            obj.weight_slack = weight_slack;
        else
            error("Default slack weight should be a scalar value. %s", ...
                "If you want to specify different slack weights for ", ...
                "each constraint, pass it to the controllers as ", ...
                "additional argument 'weight_slack'");
        end
    end
%     if isempty(obj.weight_slack)
%         error("Either params.weight.slack or params.weight_slack should be provided.");
%     end
    
    %% Do sanity check if all necessary functions are set up properly.
    x_test = zeros(obj.xdim, 1);
    % Vector fields.
    % Error should occure if they are not set properly.
    f_test = obj.f(x_test);
    g_test = obj.g(x_test);
    if size(f_test, 1) ~= obj.xdim || size(f_test, 2) ~= 1
        error("f has wrong size.");
    elseif size(g_test, 1) ~= obj.xdim || size(g_test, 2) ~= obj.udim
        error("g has wrong size.");
    end
    
    if obj.n_clf >= 1
        clf_test = obj.clf(x_test);
        if numel(clf_test) ~= obj.n_clf
            error("clf has wrong size.");
        end
        lf_clf_test = obj.lf_clf(x_test);
        if numel(lf_clf_test) ~= obj.n_clf
            error("lf_clf has wrong size.");
        end
        lg_clf_test = obj.lg_clf(x_test);
        if size(lg_clf_test, 1) ~= obj.n_clf || size(lg_clf_test, 2) ~= obj.udim
            error("lg_clf has wrong size.");
        end
    end

    if obj.n_cbf >= 1
        cbf_test = obj.cbf(x_test);
        if numel(cbf_test) ~= obj.n_cbf
            error("cbf has wrong size.");
        end
        lf_cbf_test = obj.lf_cbf(x_test);
        if numel(lf_cbf_test) ~= obj.n_cbf
            error("lf_cbf has wrong size.");
        end
        lg_cbf_test = obj.lg_cbf(x_test);
        if size(lg_cbf_test, 1) ~= obj.n_cbf || size(lg_cbf_test, 2) ~= obj.udim
            error("lg_cbf has wrong size.");
        end
    end    
    
    fprintf(obj.get_dynsys_summary())
    obj.is_sys_initialized = true;
end
function mask = get_mask(constraint_name, params, n_total)
    % Mask Parameter
    if n_total == 0
        mask = [];
        return
    end
    if isfield(params, 'active_constraint') &&...
            isfield(params.active_constraint, constraint_name)
        mask = params.active_constraint.(constraint_name);
        if max(mask) > n_total
            error("All constraint indices should be smaller than the number of pre-defined constraint");
        end
    else
        mask = 1:n_total;
    end
end