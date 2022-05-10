function [u, extraout] = ctrlFeedbackLinearizePhase(obj, t, x, mu, verbose, extraout)
%% [u, extraout] = ctrlFeedbackLinearize(obj, x, mu, verbose, extraout)
%% Implementation of feedback linearization structure when output_option=='phase'
% This function is intened to be used internally in the class function
% ctrlFeedbackLinearize
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
if nargin < 5
    verbose = 0;
end
if  nargin < 6
    extraout = [];
end

[y, dy, ~, ~, ~] = obj.eval_y(x);
if obj.rel_deg_y == 1
    Lfy = obj.lf_y(x);
    Lgy = obj.lg_y(x);
    if rank(Lgy) < obj.ydim
        error("Feedback Linearization fail because lg_y is not invertible")
    end
    feedforward = -Lgy\Lfy;
    u_raw = feedforward + Lgy\mu;
    
elseif obj.rel_deg_y == 2
    LgLfy = obj.lglf_y(x);
    L2f_y = obj.l2f_y(x);
    if rank(LgLfy) < obj.ydim
        error("Feedback Linearization fail because lglf_y is not invertible")
    end
    feedforward = -LgLfy\L2f_y;
    u_raw = feedforward + LgLfy\mu;
end

u = obj.clipInput(u_raw);



extraout.mu = mu;
extraout.y = y;
extraout.dy = dy;
extraout.feedforward = feedforward;
extraout.u_raw = u_raw;