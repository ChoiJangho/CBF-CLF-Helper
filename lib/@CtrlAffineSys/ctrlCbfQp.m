function [u, extraout] = ctrlCbfQp(obj, x, varargin)
%% [u, extraout] = ctrlCbfQp(obj, x, varagin)
%% Implementation of vanilla CBF-QP
% Inputs:   x: state
%   varargin:
%           u_ref: reference control input
%           verbose: flag for logging (1: print log, 0: run silently)
% Outputs:  u: control input as a solution of the CBF-CLF-QP
%   extraout:
%           slack: slack variable for relaxation. (empty when with_slack=0)
%           Bs: CBF values at current state.
%           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
%           when qp is infeasible, u is determined from quadprog.)
%           compt_time: computation time to run the solver.
% Author: Jason Choi (jason.choi@berkeley.edu)

    if obj.n_cbf == 0
        error('CBF is not set up so ctrlCbfQp cannot be used.');
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
    if ~isfield(kwargs, 'weight_slack')
        weight_slack = obj.weight_slack * ones(obj.n_cbf);
    else
        if numel(kwargs.weight_slack) ~= obj.n_cbf
            error("wrong weight_slack size. it should be a vector of length obj.n_cbf.");
        end
        weight_slack = kwargs.weight_slack;
    end
    if size(u_ref, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end                
            
    tstart = tic;
    Bs = obj.cbf(x);
    LfBs = obj.lf_cbf(x);
    LgBs = obj.lg_cbf(x);
        
    %% Constraints : A * u <= b
    % CBF constraint.
    A = -LgBs;
    b = LfBs + obj.cbf_rate * Bs;
    if ~isempty(obj.u_max)
        A = [A; eye(obj.udim)];
        b = [b; obj.u_max];
    end
    if ~isempty(obj.u_min)
        A = [A; -eye(obj.udim)];
        b = [b; -obj.u_min];
    end
    if with_slack
        A_slack = -eye(obj.n_cbf);
        if ~isempty(obj.u_max)
            A_slack = [A_slack; zeros(obj.udim, obj.n_cbf)];
        end
        if ~isempty(obj.u_min)
            A_slack = [A_slack; zeros(obj.udim, obj.n_cbf)];
        end
        A = [A, A_slack];
    end

    if verbose
        options =  optimset('Display','notify');
    else
        options =  optimset('Display','off');
    end

    if with_slack
        % cost = 0.5 [u' slack] H [u; slack] + f [u; slack]
        H = [obj.weight_input, zeros(obj.udim, obj.n_cbf);
            zeros(obj.n_cbf, obj.udim), diag(weight_slack)];
        f_ = [-obj.weight_input * u_ref; zeros(obj.n_cbf, 1)];
        [u_slack, ~, exitflag, ~] = quadprog(H, f_, A, b, [], [], [], [], [], options);
        if exitflag == -2            
            feas = 0;
            if verbose
                disp("Infeasible QP. Numerical error might have occured.");
            end
            u = zeros(obj.udim, 1);
            % Making up best-effort heuristic solution, if single cbf
            % constraint.
            if obj.n_cbf == 1
                for i = 1:obj.udim
                    u(i) = obj.u_min(i) * (LgBs(i) <= 0) + obj.u_max(i) * (LgBs(i) > 0);
                end
            end
            slack = zeros(obj.n_clf, 1);
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
    extraout.Bs = Bs;
    extraout.feas = feas;
    extraout.comp_time = comp_time;
end