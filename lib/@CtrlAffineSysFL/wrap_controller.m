function [u, extra_t] = wrap_controller(obj, x, mu_ref, with_slack, verbose, controller)
% Wrapper Function for feedback linearizing system
% Note
%   Created in the notion that every control system would have this wrapper
%   to be compatible with rollout_controller function
%   But this makes it impossible to extract other information that needs
%   system to evolve.
%   ex) dV_hat = V(t+1) - V(t) / dt 
%   Suggestion:
%       Instead of explicitly stating ode_func, define a conceptual
%       function like 'evolve_system' and this function might return
%       extra_t that can parse additional information. This would keep the
%       concept consistent and also can expand it by adding function like
%       'wrap_evolve_system'.
% Input
%   x
%   mu
%   extra_t: slack, feas, Vs
% Output
%   u : feedback-linearized u
%   extra_t: slack, feas, mu, y, dy, V

% TODO: going to support 
[mu, extra_t] = controller(x, mu_ref, with_slack, verbose);
u = obj.ctrlFeedbackLinearize(x, mu);
[y, dy, ~, ~, ~] = obj.eval_y(x);

extra_t.mu = mu;
extra_t.y = y;
extra_t.dy = dy;
% extra_t recording section
end
