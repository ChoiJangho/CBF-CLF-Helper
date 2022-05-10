function [u, extraout] = ctrlFeedbackLinearize(obj, t, x, mu, varargin)
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
    [mu_, extraout] = mu(t, x, varargin{:});
elseif isa(mu, 'numeric')
    mu_ = mu;
    extraout = [];
else
    error("Unknown mu type. mu should be either the numeric vector or %s", ...
        "the function handle that defines the controller.")
end

if strcmp(obj.output_option, 'phase')
    [u, extraout] = obj.ctrlFeedbackLinearizePhase(t, x, mu_, verbose, extraout);
elseif strcmp(obj.output_option, 'siso')
    [u, extraout] = obj.ctrlFeedbackLinearizeSiso(t, x, mu_, verbose, extraout);
elseif strcmp(obj.output_option, 'mimo')
    [u, extraout] = obj.ctrlFeedbackLinearizeMimo(t, x, mu_, verbose, extraout);
end
    