function [fig, Eps, Bs, Vs, cbf_constraints] = plot_cart_pole_core_results(xs, us, ts, dynsys, title_str, plot_full_state)
if nargin < 5
    title_str = [];
end
if nargin < 6
    plot_full_state = false;
end
fig = open_figure('font_size', 15, 'size', [1200, 1800]);
% Evaluate linear momentum and total energy
Eps = zeros(size(ts));
Bs = zeros(size(ts));
Vs = zeros(size(ts));
cbf_constraints = zeros(size(ts));

for i = 1:length(ts)
    Eps(i) = dynsys.pole_energy(xs(:, i));
    Bs(i) = dynsys.cbf(xs(:, i));
    Vs(i) = dynsys.clf(xs(:, i));
    % The cbf constraint is the model-based estimate that is actually used
    % in the CBF-QP.
    cbf_constraints(i) = dynsys.dcbf(xs(:, i), us(:, i)) + dynsys.cbf_rate * dynsys.cbf(xs(:, i));
end

if plot_full_state
    num_plots = 9;
else
    num_plots = 7;
end

subplot(num_plots, 1, 1);
plot(ts, xs(1, :));
hold on;
line([ts(1), ts(end)], dynsys.params.x_lim * ones(1, 2), 'Color', 'r');
line([ts(1), ts(end)], -dynsys.params.x_lim * ones(1, 2), 'Color', 'r');
ylabel('$x_1$ ($s$)');
grid on;

if ~isempty(title_str)
    title(title_str);
end

subplot(num_plots, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$ ($\theta$)');
grid on;
last_plot_index = 2;

if plot_full_state
    subplot(num_plots, 1, 3);
    plot(ts, xs(3, :));
    ylabel('$x_3$ ($\dot{s}$)');
    grid on;

    subplot(num_plots, 1, 4);
    plot(ts, xs(4, :));
    ylabel('$x_4$ ($\dot{\theta}$)');
    grid on;
    last_plot_index = 4;
end
subplot(num_plots, 1, last_plot_index+1);
plot(ts, us);
ylabel('$u$');
grid on;

subplot(num_plots, 1, last_plot_index+2);
plot(ts, Eps);
hold on;
line([ts(1), ts(end)], dynsys.potential_energy_upright() * ones(1, 2), 'Color', 'r');
ylabel('$E_p(t)$ (Pendulum Energy)');
grid on;

subplot(num_plots, 1, last_plot_index+3);
plot(ts, Vs);
ylabel('$V(x(t))$ (CLF)');
grid on;

subplot(num_plots, 1, last_plot_index+4);
plot(ts, Bs);
ylabel('$B(x(t))$ (CBF)');
grid on;

subplot(num_plots, 1, last_plot_index+5);
plot(ts, cbf_constraints);
ylabel('$\tilde{\dot{B}}(x,u) + \gamma B(x)$');
xlabel('$t$');
grid on;
end