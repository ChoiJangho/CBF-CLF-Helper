function [u, extraout] = ctrlCbfClfQp(obj, x, varargin)
%% [u, extraout] = ctrlCbfClfQp(obj, x, varagin)
%% Implementation of the vanilla CBF-CLF-QP
% Inputs:   x: state
%   varargin:
%           u_ref: reference control input
%           with_slack: flag for relaxing the clf constraint(1: relax, 0: hard-constraint)
%           verbose: flag for logging (1: print log, 0: run silently)
% Outputs:  u: control input as a solution of the CBF-CLF-QP
%   extraout:
%           slack: slack variable for relaxation. (empty when with_slack=0)
%           Bs: CBF values at the current state.
%           Vs: CLF values at the current state.
%           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
%           when qp is infeasible, u is determined from quadprog.)
%           comp_time: computation time to run the solver.
% Author: Jason Choi (jason.choi@berkeley.edu)
    if obj.n_clf == 0
        error('CLF is not set up so ctrlCbfClfQp cannot be used.');
    end
    if obj.n_cbf == 0
        error('CBF is not set up so ctrlCbfClfQp cannot be used.');
    end
        
    kwargs = parse_function_args(varargin{:});
    if ~isfield(kwargs, 'u_ref')
        % If u_ref is given, CLF-QP minimizes the norm of u-u_ref        
        % Default reference control input is u.
        u_ref = zeros(obj.udim, 1);
    else
        u_ref = kwargs.u_ref;
    end
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

    if size(u_ref, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end                

    tstart = tic;
    Vs = obj.clf(x);
    LfVs = obj.lf_clf(x);
    LgVs = obj.lg_clf(x);

    Bs = obj.cbf(x);
    LfBs = obj.lf_cbf(x);
    LgBs = obj.lg_cbf(x);
        
    %% Constraints: A[u; slack] <= b
    % CLF and CBF constraints.
    A = [LgVs; -LgBs];
    b = [-LfVs - obj.clf_rate * Vs;
        LfBs + obj.cbf_rate * Bs]; 
    % Input constraints
    if ~isempty(obj.u_max)
        A = [A; eye(obj.udim)];
        b = [b; obj.u_max];
    end
    if ~isempty(obj.u_min)
        A = [A; -eye(obj.udim)];
        b = [b; -obj.u_min];
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
        f_ = [-obj.weight_input * u_ref; zeros(n_slack, 1)];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            if verbose
                disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
            end
            u = zeros(obj.udim, 1);
            slack = zeros(n_slack, 1);
        else
            feas = 1;
            u = u_slack(1:obj.udim);
            slack = u_slack(obj.udim+1:end);
        end        
    else
        % cost = 0.5 u' H u + f u
        H = obj.weight_input;
        f_ = -obj.weight_input * u_ref;
        [u, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2
            feas = 0;
            if verbose
                disp("Infeasible QP. CBF constraint is conflicting with input constraints.");
            end
        else
            feas = 1;
        end
        slack = [];
    end
    comp_time = toc(tstart);
    
    extraout.slack = slack;
    extraout.Vs = Vs;
    extraout.Bs = Bs;
    extraout.feas = feas;
    extraout.comp_time = comp_time;
end