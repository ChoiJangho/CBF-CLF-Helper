function [fig, Eps, Bs, Vs, cbf_constraints] = plot_cart_pole_core_results(xs, us, ts, dynsys, ...
    title_str, extras, train_data, plot_full_state)

if nargin < 5
    title_str = [];
end
if nargin < 6
    extras = [];
end
if nargin < 7
    train_data = [];
end

if nargin < 8
    plot_full_state = false;
end
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

if plot_full_state && ~isempty(extras)
    num_plots = 11;
elseif ~isempty(extras)
    num_plots = 9;
elseif plot_full_state
    num_plots = 9;
else
    num_plots = 7;
end
fig = open_figure('font_size', 15, 'size', [1200, num_plots * 200]);
tile = tiledlayout(num_plots, 1);
if ~isempty(title_str)
    title(tile, title_str);
end

nexttile;
plot(ts, xs(1, :));
hold on;
line([ts(1), ts(end)], dynsys.params.x_lim * ones(1, 2), 'Color', 'r');
line([ts(1), ts(end)], -dynsys.params.x_lim * ones(1, 2), 'Color', 'r');
if ~isempty(train_data)
    scatter(train_data.ts_train, train_data.xs_train(1, :), 'x');
end
ylabel('$x_1$ ($s$)');
grid on;
xlim([ts(1), ts(end)]);

nexttile;
plot(ts, xs(2, :));
if ~isempty(train_data)
    hold on;
    scatter(train_data.ts_train, train_data.xs_train(2, :), 'x');
end
ylabel('$x_2$ ($\theta$)');
grid on;
xlim([ts(1), ts(end)]);

if plot_full_state
    nexttile;
    plot(ts, xs(3, :));
    if ~isempty(train_data)
        hold on;
        scatter(train_data.ts_train, train_data.xs_train(3, :), 'x');
    end
    ylabel('$x_3$ ($\dot{s}$)');
    grid on;
    xlim([ts(1), ts(end)]);

    nexttile;
    plot(ts, xs(4, :));
    if ~isempty(train_data)
        hold on;
        scatter(train_data.ts_train, train_data.xs_train(4, :), 'x');
    end
    ylabel('$x_4$ ($\dot{\theta}$)');
    grid on;
    xlim([ts(1), ts(end)]);
end

nexttile;
plot(ts, us);
if ~isempty(train_data)
    hold on;
    scatter(train_data.ts_train, train_data.ys_train(2, :), 'x');
end
ylabel('$u$');
grid on;
xlim([ts(1), ts(end)]);

nexttile;
plot(ts, Eps);
hold on;
line([ts(1), ts(end)], dynsys.potential_energy_upright() * ones(1, 2), 'Color', 'r');
ylabel('$E_p(t)$ (Pendulum Energy)');
grid on;
xlim([ts(1), ts(end)]);

nexttile;
plot(ts, Vs);
ylabel('$V(x(t))$ (CLF)');
grid on;
xlim([ts(1), ts(end)]);

nexttile;
plot(ts, Bs);
ylabel('$B(x(t))$ (CBF)');
grid on;
xlim([ts(1), ts(end)]);

nexttile;
plot(ts, cbf_constraints);
ylabel('$\tilde{\dot{B}}(x,u) + \gamma B(x)$');
grid on;
xlim([ts(1), ts(end)]);

if ~isempty(extras)
    nexttile;
    plot(ts, extras.feas);
    ylabel('Feasibility')
    grid on;
    xlim([ts(1), ts(end)]);

    nexttile;
    plot(ts, 1000 * extras.comp_times);
    ylabel('Comp. Time [ms]');
    ylim([0, 50]);
    grid on;
    xlim([ts(1), ts(end)]);
end
xlabel('$t$');

end