function [u, extraout] = ctrlClfSontag(obj, x, varargin)
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

% Lie derivatives of the CLF.
V = obj.clf(x);
LfV = obj.lf_clf(x);
LgV = obj.lg_clf(x);

if LgV ~= 0
    u_raw = - (LfV + sqrt(LfV^2 + LgV^4)) / LgV;
else
    u_raw = 0;
end
u = obj.clipInput(u_raw);
extraout.u_raw = u_raw;
extraout.Vs = V;