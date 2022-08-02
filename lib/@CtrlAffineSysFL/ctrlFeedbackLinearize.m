function [u, extraout] = ctrlFeedbackLinearize(obj, x, mu, varargin)
%% [u, extraout] = ctrlFeedbackLinearize(obj, x, mu, varargin)
%% Implementation of feedback linearization structure
% Inputs:   x: state
%           mu: virtual control input
%               mu can be a vector of (obj.udim, 1) or it can also be a
%               function handle.
%           varargin: extra inputs (only necessary when mu is a handle.)
% Outputs:  u: control input.
% extraout: y: output (defined for feedback linearization).
%           feedforward: feedforward term of the control
%           u_raw: control input before clipped by the input saturation
%           constraint.
%           When mu is a function handle, it might contain other fields
%           that are from the controller that defines mu.
% u = u_star + LgLf\mu
kwargs = parse_function_args(varargin{:});
if ~isfield(kwargs, 'verbose')
    % Run QP without log in default condition.
    verbose = 0;
else
    verbose = kwargs.verbose;
end
    
if isa(mu, 'function_handle')
    [mu_, extraout] = mu(x, varargin{:});
elseif isa(mu, 'numeric')
    mu_ = mu;
else
    error("Unknown mu type. mu should be either the numeric vector or %s", ...
        "the function handle that defines the controller.")
end

[y, dy, ~, ~, ~] = obj.eval_y(x);
if obj.rel_deg_y == 1
    Lfy = obj.lf_y(x);
    Lgy = obj.lg_y(x);
    if rank(Lgy) < obj.ydim
        error("Feedback Linearization fail because lg_y is not invertible")
    end
    feedforward = -Lgy\Lfy;
    u_raw = feedforward + Lgy\mu_;
    
elseif obj.rel_deg_y == 2
    LgLfy = obj.lglf_y(x);
    L2f_y = obj.l2f_y(x);
    if rank(LgLfy) < obj.ydim
        error("Feedback Linearization fail because lglf_y is not invertible")
    end
    feedforward = -LgLfy\L2f_y;
    u_raw = feedforward + LgLfy\mu_;    
end

u = u_raw;

%% Clip input
if ~isempty(obj.u_max)
    for i = 1:obj.udim
        if u(i) > obj.u_max(i)
            if verbose
                disp("Warning: virtual control input does not satisfy input constraint.")
            end
            u(i) = obj.u_max(i);
        end    
    end
end

if ~isempty(obj.u_min)
    for i = 1:obj.udim
        if u(i) < obj.u_min(i)
            if verbose
                disp("Warning: virtual control input does not satisfy input constraint.")
            end
            u(i) = obj.u_min(i);
        end    
    end
end

extraout.mu = mu_;
extraout.y = y;
extraout.dy = dy;
extraout.feedforward = feedforward;
extraout.u_raw = u_raw;