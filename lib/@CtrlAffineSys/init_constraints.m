function init_constraints(obj, params, varargin)

kwargs = parse_function_args(varargin{:});
init_clf = 1;
if isfield(kwargs, 'init_clf')
    init_clf = kwargs.init_clf;
end
init_cbf = 1;
if isfield(kwargs, 'init_cbf')
    init_cbf = kwargs.init_cbf;
end
init_additional = 1;
if isfield(kwargs, 'init_additional')
    init_additional = kwargs.init_additional;
end

if strcmp(obj.setup_option, 'symbolic') 
    [x, f_, g_] = obj.defineSystem(params);
    if ~isa(f_, 'sym')
        f_ = sym(f_);
    end
    if ~isa(g_, 'sym')
        g_ = sym(g_);
    end
    if init_clf
        % Retrieve clf that is defined.
        clf_ = obj.defineClf(params, x);
        if ~isempty(clf_)
            init_clf_constraints_symbolic(obj, x, f_, g_, clf_);
        else
            obj.n_clf = 0;
        end
    end
    
    if init_cbf
        % Retrieve cbf that is defined.
        cbf_ = obj.defineCbf(params, x);
        if ~isempty(cbf_)
            init_cbf_constraints_symbolic(obj, x, f_, g_, cbf_);
        else
            obj.n_cbf = 0;
        end
    end
elseif strcmp(obj.setup_option, 'built-in')
    x_test = zeros(obj.xdim, 1);
    if init_clf
        try
            clf_test = obj.clf_all(x_test);
            n_clf = length(clf_test);
        catch e
            n_clf = 0;
        end
        if n_clf > 0
            lf_clf_test = obj.lf_clf_all(x_test);
            if numel(lf_clf_test) ~= n_clf
                error("lf_clf_all has wrong size.");
            end
            lg_clf_test = obj.lg_clf_all(x_test);
            if size(lg_clf_test, 1) ~= n_clf || size(lg_clf_test, 2) ~= obj.udim
                error("lg_clf_all has wrong size.");
            end
            obj.clf_builtin = @(x) obj.clf_all(x);
            obj.lf_clf_builtin = @(x) obj.lf_clf_all(x);
            obj.lg_clf_builtin = @(x) obj.lg_clf_all(x);
        end
        obj.n_clf = n_clf;
    end

    if init_cbf
        try
            cbf_test = obj.cbf_all(x_test);
            n_cbf = length(cbf_test);
        catch e
            n_cbf = 0;
        end
        if n_cbf > 0
            lf_cbf_test = obj.lf_cbf_all(x_test);
            if numel(lf_cbf_test) ~= obj.n_cbf
                error("lf_cbf has wrong size.");
            end
            lg_cbf_test = obj.lg_cbf_all(x_test);
            if size(lg_cbf_test, 1) ~= n_cbf || size(lg_cbf_test, 2) ~= obj.udim
                error("lg_cbf has wrong size.");
            end            
            obj.cbf_builtin = @(x) obj.cbf_all(x);
            obj.lf_cbf_builtin = @(x) obj.lf_cbf_all(x);
            obj.lg_cbf_builtin = @(x) obj.lg_cbf_all(x);
        end
        obj.n_cbf = n_cbf;
    end
else
    error("Undefined setup_option.");
end

% set cbf rate, clf rate
if init_clf
    if isfield(params, 'clf') && isfield(params.clf, 'rate')
        clf_rate = params.clf.rate;
    elseif isfield(params, 'clf_rate')
        clf_rate = params.clf_rate;
    elseif obj.n_clf >= 1
        error("params.clf.rate or params.clf_rate should be provided %s", ...
            "in order to use CLF.");
    end
    if obj.n_clf >= 1
        if length(clf_rate) == 1
            obj.clf_rate = clf_rate * ones(obj.n_clf, 1);
        elseif length(clf_rate) ~= obj.n_clf
            error("Invalid size of params.clf.rate");
        else
            if isrow(clf_rate)
                obj.clf_rate = clf_rate';
            else
                obj.clf_rate = clf_rate;
            end
        end    
    end
    
    %% Initialize constraints mask
    % By default, all clf and cbf constraints are active.
    % By default, all clf and cbf constraints are soft constraints
    if obj.n_clf >= 1
        obj.clf_active_mask = 1:obj.n_clf;
        obj.n_clf_active = length(obj.clf_active_mask);
        obj.clf_slack_mask = 1:obj.n_clf;
    end
end

if init_cbf
    if isfield(params, 'cbf') && isfield(params.cbf, 'rate')
        cbf_rate = params.cbf.rate;
    elseif isfield(params, 'cbf_rate')
        cbf_rate = params.cbf_rate;
    elseif obj.n_cbf >= 1
        error("params.cbf.rate or params.cbf_rate should be provided %s", ...
            "in order to use CBF.");
    end
    if obj.n_cbf >= 1
        if length(cbf_rate) == 1
            obj.cbf_rate = cbf_rate * ones(obj.n_cbf, 1);
        elseif length(cbf_rate) ~= obj.n_cbf
            error("Invalid size of params.cbf.rate");
        else
            if isrow(cbf_rate)
                obj.cbf_rate = cbf_rate';
            else
                obj.cbf_rate = cbf_rate;
            end
        end    
    end
    %% Initialize constraints mask
    % By default, all clf and cbf constraints are active.
    % By default, all clf and cbf constraints are soft constraints
    if obj.n_cbf >= 1
        obj.cbf_active_mask = 1:obj.n_cbf;
        obj.n_cbf_active = length(obj.cbf_active_mask);
        obj.cbf_slack_mask = 1:obj.n_cbf;
    end
end

%% TODO: Add initialization of additional constraints.

fprintf(obj.get_constraints_summary());
end

function init_clf_constraints_symbolic(obj, x, f_, g_, clf_)
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
    end
    obj.clf_sym = clf_sym;             
    obj.lf_clf_sym = lf_clf_sym;
    obj.lg_clf_sym = lg_clf_sym;
end

function init_cbf_constraints_symbolic(obj, x, f_, g_, cbf_)
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
    lf_cbf_sym_str = cell(obj.n_cbf, 1);
    lg_cbf_sym_str = cell(obj.n_cbf, 1);
    syms('cbf_sym_array', [obj.n_cbf, 1]);
    syms('lf_cbf_', [obj.n_cbf, 1]);
    syms('lg_cbf_', [obj.n_cbf, 1]);

    for i_cbf = 1:obj.n_cbf
        cbf_sym{i_cbf} = matlabFunction(cbf_{i_cbf}, 'vars', {x});

        dcbf_i = simplify(jacobian(cbf_{i_cbf}, x));
        lf_cbf_i = dcbf_i * f_;
        lg_cbf_i = dcbf_i * g_;        
        lf_cbf_sym{i_cbf} = matlabFunction(lf_cbf_i, 'vars', {x});
        lf_cbf_sym_str{i_cbf} = string(lf_cbf_i);
        if all(isAlways(simplify(lg_cbf_i) == 0, 'Unknown', 'false'))
            error('Relative degree of the defined %d-th CBF > 1. %s', ...
            [i_cbf, 'Currently, High-order relative degree is not supported.']);
        end
        lg_cbf_sym{i_cbf} = matlabFunction(lg_cbf_i, 'vars', {x});
        lg_cbf_sym_str{i_cbf} = string(lg_cbf_i);
    end
    obj.cbf_sym = cbf_sym;             
    obj.lf_cbf_sym = lf_cbf_sym;
    obj.lg_cbf_sym = lg_cbf_sym;
    obj.lf_cbf_sym_str = lf_cbf_sym_str;
    obj.lg_cbf_sym_str = lg_cbf_sym_str;
end