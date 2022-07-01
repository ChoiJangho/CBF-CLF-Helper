%% Author: Jason Choi (jason.choi@berkeley.edu)
function [mu, extraout] = ctrlCbfClfQpFL(obj, t, x, varargin)
    %% Implementation of CBF-CLF-QP under feedback linearization structure.
    % Inputs:   s: state
    %           mu_ref: reference virtual control input
    %           with_slack: flag for relaxing the clf constraint(1: relax, 0: hard-constraint)
    %           verbose: flag for logging (1: print log, 0: run silently)
    % Outputs:  u: control input as a solution of the CBF-CLF-QP
    %           slack: slack variable for relaxation. (empty list when with_slack=0)
    %           B: Value of the CBF at current state.
    %           V: Value of the CLF at current state.
    %           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
    %           when qp is infeasible, u is determined from quadprog.)
    
    % TODO: AffinesystemFL => u_ref, mu_ref (varargin input!)
    kwargs = parse_function_args(varargin{:});
    
    % QP minimizes the norm of mu-mu_ref
    if ~isfield(kwargs, 'mu_ref')            
        % Default reference control input is 0.
        mu_ref_ = zeros(obj.udim, 1);
        mu_ref = mu_ref_;
    else
        mu_ref = kwargs.mu_ref;
        if isa(mu_ref, 'function_handle')
            [mu_ref_, extraout] = mu_ref(t, x, varargin{:});
        elseif isa(mu_ref, 'numeric')
            mu_ref_ = mu_ref;
        elseif isa(mu_ref, 'char')
            if ~strcmp(mu_ref, 'min_ctrl_diff')
                error("Currently, mu_ref as string option only supports 'min_ctrl_diff'");
            end
            % 'ctrl_prev' should be provided as extra argument in order to use 'min_ctrl_diff' option.
            % otherwise, it assumes that it's the very beginning of the
            % simulation and use mu_prev = zeros(obj.udim, 1);
            mu_prev = zeros(obj.udim, 1);
            if isfield(kwargs, 'ctrl_prev')
                mu_prev = kwargs.ctrl_prev;
            end
            %% determines the portion of |mu - mu_prev| weight compared to |mu|
            % 1: minimizes |mu - mu_prev|^2
            % 0: minimizes |mu|^2
            ratio_ctrl_diff = 1;
            if isfield(kwargs, 'ratio_ctrl_diff')
                if ratio_ctrl_diff > 1 || ratio_ctrl_diff < 0
                    error("ratio_ctrl_diff should be a value between 0 and 1.");
                end                                
                ratio_ctrl_diff = kwargs.ratio_ctrl_diff;
            end
            mu_ref_ = ratio_ctrl_diff * mu_prev;
        else
            error("Unknown mu_ref type.");
        end
    end
    extraout.mu_ref = mu_ref_;
    
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

    if size(mu_ref, 1) ~= obj.udim
        error("Wrong size of mu_ref, it should be (udim, 1) array.");
    end      
            
    tstart = tic;
    [y, dy, L2fy, LgLfy, ~] = obj.eval_y(x);
    
    V = obj.clf_FL(y, dy);
    LfV = obj.lF_clf_FL(y, dy);
    LgV = obj.lG_clf_FL(y, dy);

    Bs = obj.cbf(x);
    LfBs = obj.lf_cbf(x);
    LgBs = obj.lg_cbf(x);
        
    inv_LgLfy = inv(LgLfy);
    u_star = -LgLfy\L2fy; %feedforward term

    
    %% Constraint : A[u; slack] <= b
    A = [LgV; -LgBs];
    b = [-LfV - obj.clf_rate * V;
        LfBs + obj.cbf_rate * Bs];
    if ~isempty(obj.u_max)
        A = [A; inv_LgLfy];
        b = [b; obj.u_max - u_star];
    end
    if ~isempty(obj.u_min)
        A = [A; -inv_LgLfy];
        b = [b; u_star - obj.u_min];
    end
    if with_slack
        % n_slack(size of slack):
        %   = n_clf if n_cbf=1 (Relaxing only the CLF constraints)
        %   = (n_clf + n_cbf) if n_cbf >1 (Relaxing all constraints)
        if obj.n_cbf == 1
            n_slack = obj.n_clf;
            A_slack = [-eye(obj.n_clf); zeros(1, obj.n_clf)];
        else
            n_slack = obj.n_clf + obj.n_cbf;
            A_slack = -eye(obj.n_clf + obj.n_cbf);
        end
        if ~isempty(obj.u_max)
            A_slack = [A_slack; zeros(obj.udim, n_slack)];
        end
        if ~isempty(obj.u_min)
            A_slack = [A_slack; zeros(obj.udim, n_slack)];
        end
        A = [A, A_slack];
    end
    
    if ~isfield(kwargs, 'weight_slack')
        weight_slack = obj.weight_slack * ones(n_slack, 1);
    else
        if numel(kwargs.weight_slack) ~= n_slack
            error("wrong weight_slack size. it should be a vector of n_slack:=%s", ...
                "n_clf if n_cbf==1, else (n_clf+n_cbf).");
        end
        weight_slack = kwargs.weight_slack;
    end
    
        
    if verbose
        options =  optimset('Display','notify');
    else
        options =  optimset('Display','off');
    end
    
    if with_slack         
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [obj.weight_input, zeros(obj.udim, n_slack);
            zeros(n_slack, obj.udim), diag(weight_slack)];
        f_ = [-obj.weight_input * mu_ref; zeros(n_slack, 1)];
        [mu_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            if verbose
                disp("Infeasible QP. Numerical error might have occured.");
            end
            mu = zeros(obj.udim ,1);
            slack = 0;
        else
            feas = 1;
            mu = mu_slack(1:obj.udim);
            slack = mu_slack(end);
        end
    else
        % cost = 0.5 u' H u + f u
        H = obj.weight_input;
        f_ = -obj.weight_input * mu_ref;
        [mu, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            if verbose
                disp("Infeasible QP. constraints are conflicting with input constraints.");
            end
            mu = zeros(obj.udim, 1);
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
    
    extraout.slack = slack;
    extraout.feas = feas;
    extraout.Vs = V;
    extraout.Bs = Bs;
    extraout.comp_time = comp_time;
    if isa(mu_ref, 'char') && strcmp(mu_ref, 'min_ctrl_diff')
        extraout.ctrl_prev = mu;
    end    
end