function init_sys(obj, params, init_constraints)
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
        
        obj.f_sym = cell(obj.xdim, 1);        
        for i = 1:obj.xdim
            if isSymType(f_(i), 'constant')
                f_i = double(f_(i));                
                obj.f_sym{i} = @(x) f_i * ones(1, size(x, 2));
            else
                obj.f_sym{i} = matlabFunction(f_(i), 'vars', {x});
            end
        end
        % obj.f_sym = matlabFunction(f_, 'vars', {x});
%         obj.g_sym = matlabFunction(g_, 'vars', {x});
        obj.g_sym = cell(obj.xdim, obj.udim);        
        for i = 1:obj.xdim
            for j = 1:obj.udim
                if isSymType(g_(i, j), 'constant')
                    g_ij = double(g_(i, j));
                    obj.g_sym{i, j} = @(x) g_ij * ones(1, size(x, 2));
                else
                    obj.g_sym{i, j} = matlabFunction(g_(i, j), 'vars', {x});
                end
            end
        end        
        
        if generate_clone
            f_file_str = fullfile(clone_class_path, 'f');
            matlabFunction(f_, 'file', f_file_str, 'vars', {obj_, x});
            g_file_str = fullfile(clone_class_path, 'g');
            matlabFunction(g_, 'file', g_file_str, 'vars', {obj_, x});
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
    else
        error("Undefined setup_option.");
    end
    
    if isfield(params, 'dims_angle')
        obj.dims_angle = logical(params.dims_angle);
    end

    
    %% Parse parameters for both setup_option.    
    if isfield(params, 'u_min')
        if length(params.u_min) == 1
            obj.u_min_constant = params.u_min * ones(obj.udim, 1);
        elseif length(params.u_min) ~= obj.udim
            error("Invalid size of params.u_min.");
        else
            if isrow(params.u_min)
                obj.u_min_constant = params.u_min';
            else
                obj.u_min_constant = params.u_min;
            end
        end
    end
    if isfield(params, 'u_max')
        if length(params.u_max) == 1
            obj.u_max_constant = params.u_max * ones(obj.udim, 1);
        elseif length(params.u_max) ~= obj.udim
            error("Invalid size of params.u_max.");
        else
            if isrow(params.u_max)
                obj.u_max_constant = params.u_max';
            else
                obj.u_max_constant = params.u_max;
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
    
    %% Do sanity check if all necessary functions are set up properly.
%     x_test = zeros(obj.xdim, 1);
    % Vector fields.
    % Error should occure if they are not set properly.
%     f_test = obj.f(x_test);
%     g_test = obj.g(x_test);
%     if size(f_test, 1) ~= obj.xdim || size(f_test, 2) ~= 1
%         error("f has wrong size.");
%     elseif size(g_test, 1) ~= obj.xdim || size(g_test, 2) ~= obj.udim
%         error("g has wrong size.");
%     end
        
    fprintf(obj.get_dynsys_summary());
    obj.is_sys_initialized = true;
    
    if init_constraints
        obj.init_constraints(params);
    end
end
