function [u, extraout] = ctrl_clf_min_norm(obj, t, x, varargin)
%% [u, extraout] = ctrl_clf_min_norm(obj, t, x, varargin)
%% Implementation of the closed-form min-norm clf controller.
% Inputs:   x: state
%   varargin:
%           u_ref: reference control input
%           LgV_tolerance: Regarding LgB whose absolute value smaller than LgB_tolearance as numerical error. (default:
%           1e-6);
% Outputs:  u: control input as a solution of the CBF-CLF-QP
%   extraout:

% The implementation is based on Theorem 2, Alan et al., Control Barrier Functions and Input-to-State Safety
% with Application to Automated Vehicles
% Author: Jason Choi (jason.choi@berkeley.edu)

    if obj.n_clf_active == 0
        error('No active CLF constraint.');
    elseif obj.n_clf_active > 1
        error("This closed-form min-norm controller only works for one CLF constraint.");        
    end
    kwargs = parse_function_args(varargin{:});
    
    % QP minimizes the norm of u-u_ref
    if ~isfield(kwargs, 'u_ref')            
        % Default reference control input is 0.
        u_ref_ = zeros(obj.udim, 1);
        u_ref = u_ref_;
        extraout = [];
    else
        u_ref = kwargs.u_ref;
        if isa(u_ref, 'function_handle')
            varargin = obj.remove_latest_u_ref(varargin);
            [u_ref_, extraout] = u_ref(t, x, varargin{:});
            extraout = obj.append_u_ref(extraout, u_ref_);
        elseif isa(u_ref, 'numeric')
            u_ref_ = u_ref;
            extraout.u_ref = u_ref_;
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
            extraout.u_ref = u_ref_;            
        else
            error("Unknown u_ref type.");
        end
    end
    
    if ~isfield(kwargs, 'LgV_tolerance')
        LgV_tolerance = 1e-8;
    else
        LgV_tolerance = kwargs.LgV_tolerance;
    end
    
    if size(u_ref_, 1) ~= obj.udim
        error("Wrong size of u_ref, it should be (udim, 1) array.");
    end
                       
    V = obj.clf(x);
    LfV = obj.lf_clf(x);
    LgV = obj.lg_clf(x);
    b = LfV + obj.clf_rate(obj.clf_active_mask) * V;
    
    if norm(LgV) < LgV_tolerance
        if b > 0
            disp("Warning: cbf constraint might be infeasible.");
        end
        eta = zeros(obj.udim, 1);
    else
        eta = -(b + LgV * u_ref_) / norm(LgV)^2;
        eta = min(0, eta);
    end
    
    u = u_ref_ + eta * LgV';

    if isa(u_ref, 'char') && strcmp(u_ref, 'min_ctrl_diff')
        extraout.ctrl_prev = u;
    end    
end