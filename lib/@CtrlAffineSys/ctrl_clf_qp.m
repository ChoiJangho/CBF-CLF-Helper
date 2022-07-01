function [u, extraout] = ctrl_clf_qp(obj, t, x, varargin)
%% [u, extraout] = ctrlClfQp(obj, x, varagin)
%% Implementation of the vanilla CLF-QP
% Inputs:   x: state
%   varargin:
%           u_ref: reference control input
%           with_slack: flag for relaxing (1: relax, 0: hard CLF constraint)
%           verbose: flag for logging (1: print log, 0: run silently)
% Outputs:  u: control input as a solution of the CLF-QP
%   extraout:
%           slack: slack variable for relaxation. (empty when with_slack=0)
%           Vs: CLF values at the current state.
%           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
%           when qp is infeasible, u is determined from quadprog.)
%           comp_time: computation time to run the solver.
% Author: Jason Choi (jason.choi@berkeley.edu)
    if obj.n_clf == 0
        error('CLF is not set up so ctrlClfQp cannot be used.');
    end
    
    kwargs = parse_function_args(varargin{:});
    
    % QP minimizes the norm of u-u_ref
    if ~isfield(kwargs, 'u_ref')            
        % Default reference control input is 0.
        u_ref_ = zeros(obj.udim, 1);
        u_ref = u_ref_;
    else
        u_ref = kwargs.u_ref;
        if isa(u_ref, 'function_handle')
            [u_ref_, extraout] = u_ref(t, x, varargin{:});
        elseif isa(u_ref, 'numeric')
            u_ref_ = u_ref;
        elseif isa(u_ref, 'char')
            if ~strcmp(u_ref, 'min_ctrl_diff')
                error("Currently, u_ref as string option only supports 'min_ctrl_diff'");
            end
            % 'ctrl_prev' should be provided as extra argument in order to use 'min_ctrl_diff' option.
            % otherwise, it assumes that it's the very beginning of the
            % simulation and use u_prev = zeros(obj.udim, 1);
            u_prev = zeros(obj.udim, 1);
            if isfield(kwargs, 'ctrl_prev')
                u_prev = kwargs.ctrl_prev;
            end
            %% determines the portion of |u - u_prev| weight compared to |u|
            % 1: minimizes |u - u_prev|^2
            % 0: minimizes |u|^2
            ratio_ctrl_diff = 1;
            if isfield(kwargs, 'ratio_ctrl_diff')
                if ratio_ctrl_diff > 1 || ratio_ctrl_diff < 0
                    error("ratio_u_diff should be a value between 0 and 1.");
                end                                
                ratio_ctrl_diff = kwargs.ratio_ctrl_diff;
            end
            u_ref_ = ratio_ctrl_diff * u_prev;
        else
            error("Unknown u_ref type.");
        end
    end
    extraout.u_ref = u_ref_;
    
    if ~isfield(kwargs, 'with_slack')
        % Relaxing is activated in default condition.
        with_slack = 1;
    else
        with_slack = kwargs.with_slack;
    end
    if ~isfield(kwargs, 'verbose')
        % Run QP without log in default condition.
        verbose = 0;
    else
        verbose = kwargs.verbose;
    end
    if ~isfield(kwargs, 'weight_slack')
        weight_slack = obj.weight_slack * ones(obj.n_clf);
    else
        if numel(kwargs.weight_slack) ~= obj.n_clf
            error("wrong weight_slack size. it should be a vector of length obj.n_clf.");
        end
        weight_slack = kwargs.weight_slack;        
    end
    
    if size(u_ref_, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end
    
    tstart = tic;
    Vs = obj.clf(x);
    % Lie derivatives of the CLF.
    LfVs = obj.lf_clf(x);
    LgVs = obj.lg_clf(x);

    %% Constraints : A[u; slack] <= b
    A = LgVs;
    b = -LfVs - obj.clf_rate * Vs;
    if ~isempty(obj.u_max)
        A = [A; eye(obj.udim)];
        b = [b; obj.u_max];
    end
    if ~isempty(obj.u_min)
        A = [A; -eye(obj.udim)];
        b = [b; -obj.u_min];
    end
    if with_slack
        A_slack = -eye(obj.n_clf);
        if ~isempty(obj.u_max)
            A_slack = [A_slack; zeros(obj.udim, obj.n_clf)];
        end
        if ~isempty(obj.u_min)
            A_slack = [A_slack; zeros(obj.udim, obj.n_clf)];
        end
        A = [A, A_slack];
    end
    
    if verbose
        options =  optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'StepTolerance', 1e-12, 'Display','iter');
    else
        options =  optimoptions('quadprog', 'ConstraintTolerance', 1e-6, 'Display','off');
    end
    
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [obj.weight_input, zeros(obj.udim, obj.n_clf);
            zeros(obj.n_clf, obj.udim), diag(weight_slack)];
        f_ = [-obj.weight_input * u_ref_; zeros(obj.n_clf, 1)];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2            
            feas = 0;
            if verbose
                disp("Infeasible QP. Numerical error might have occured.");
            end
            u = zeros(obj.udim, 1);
            % Making up best-effort heuristic solution, if single clf
            % constraint.
            if obj.n_clf == 1
                for i = 1:obj.udim
                    u(i) = obj.u_min(i) * (LgVs(i) > 0) + obj.u_max(i) * (LgVs(i) <= 0);
                end
            end
            slack = zeros(obj.n_clf, 1);
        else
            feas = 1;
            u = u_slack(1:obj.udim);
            slack = u_slack(obj.udim+1:end);
        end
    else
        H = obj.weight_input;
        f_ = -obj.weight_input * u_ref_;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            if verbose
                disp("Infeasible QP. CLF constraint is conflicting with input constraints.");
            end
            % Making up best-effort heuristic solution.
            u = zeros(obj.udim, 1);
            for i = 1:obj.udim
                u(i) = obj.u_min(i) * (LgVs(i) > 0) + obj.u_max(i) * (LgVs(i) <= 0);
            end
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
    
    extraout.slack = slack;
    extraout.Vs = Vs;
    extraout.feas = feas;
    extraout.comp_time = comp_time;
    if isa(u_ref, 'char') && strcmp(u_ref, 'min_ctrl_diff')
        extraout.ctrl_prev = u;
    end
end
