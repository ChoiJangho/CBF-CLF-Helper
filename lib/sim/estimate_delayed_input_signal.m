function us_delayed = estimate_delayed_input_signal(us, ts, delay_constant)
% esimate the delayed control signal
% Assumptions:
% the delay dynamics is the first-order dynamics given by
%   \dot{u_actual} = -(1 / tau) * u_actual + (1 / tau) * u
% the control signal is applied by the sample-and-hold method.
if size(us, 2) ~= length(ts)
    error("Wrong us and ts size.");
end

us_delayed = zeros(size(us));
for i = 1:length(ts)-1
    u_delayed = us_delayed(:, i);
    [~, us_temp] = ode45(@(t, u) delay_dynamics(t, u, delay_constant, us(:, i)), [0, ts(i+1) - ts(i)], u_delayed);
    us_delayed(:, i+1) = us_temp(end, :)';
end

end

function du = delay_dynamics(t, u, tau, u_signal)
    du = - (1/tau) * u + (1/tau) * u_signal;
end