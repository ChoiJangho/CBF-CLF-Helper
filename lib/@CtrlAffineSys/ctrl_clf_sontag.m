function [u, extraout] = ctrl_clf_sontag(obj, t, x, varargin)
%% [u, extraout] = ctrlClfSontag(obj, x, varagin)
%% Implementation of the vanilla CLF-QP
% Inputs:   x: state
%   varargin:
%           u_ref: reference control input
%           verbose: flag for logging (1: print log, 0: run silently)
% Outputs:  u: control input as a solution of the CLF-QP
%   extraout:
%           Vs: CLF values at the current state.
%           feas: 1 if QP is feasible, 0 if infeasible. (Note: even
%           when qp is infeasible, u is determined from quadprog.)
%           comp_time: computation time to run the solver.
% Author: Jason Choi (jason.choi@berkeley.edu)

if obj.udim > 1
    error("NotImplemented");
end
if obj.n_clf_active == 0
    error('No active CLF constraint.');
elseif obj.n_clf_active > 1
    error("This closed-form min-norm controller only works for one CLF constraint.");        
end
kwargs = parse_function_args(varargin{:});

gamma_active = false;
if isfield(kwargs, 'gamma_active')
    gamma_active = kwargs.gamma_active;
end
    
% Lie derivatives of the CLF.
V = obj.clf(x);
LfV = obj.lf_clf(x);
LgV = obj.lg_clf(x);
if gamma_active
    b = LfV + obj.clf_rate(obj.clf_active_mask) * V;
else
    b = LfV;
end

if LgV ~= 0
    u_raw = - (b + sqrt(b^2 + LgV^4)) / LgV;
else
    u_raw = 0;
end
u = obj.clipInput(u_raw);
extraout.u_raw = u_raw;
extraout.Vs = V;