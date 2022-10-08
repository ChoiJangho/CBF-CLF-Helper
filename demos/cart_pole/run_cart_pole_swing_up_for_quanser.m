clear all;
% close all;
dt = 0.01;

% Toggle to activate or deactivate CBF-QP.
use_cbf_filter = true;
introduce_delay = true;

% Choice of Model Type: options:
%   'ONES': set every model parameters to 1 (including the gravity)
%   'QUANSER': Using the full quanser parameter values
%   'QUANSER_NO_DRAG': Using the full quanser parameter values except for
%   the drag terms.
%   'QUANSER_SIMPLE': Use only the mass and the length values from Quanser.
% Choice of Quanser weight type: 'NO_LOAD', 'WEIGHT', '2WEIGHTS'
params = get_predefined_params_set_for_vanilla_cart_pole('QUANSER', 'NO_LOAD');
% Set up input saturation limit.
params.u_max = (params.m + params. M) * 10;
% Set up CLF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;
% Create the dynamic system to simulate.
dynsys_force = CartPole(params);
dynsys_force.init_constraints(dynsys_force.params);

% Set up input saturation limit.
params.u_max = (params.m + params. M) * 10;
% Set up CLF, CBF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;
params.k_cbf = 10;
params.x_lim = 0.35;
params.cbf.rate = 10;
% Set up new input bound for the Quanser dynsys.
params.u_max = 6;
params.u_min = -6;
dynsys = QuanserCartPole(params, '2WEIGHTS');
dynsys.init_constraints(dynsys.params);

controller_for_force = @(t, x, varargin) dynsys_force.ctrl_hybrid_swing_up( ...
  t, x, 'k_energy', 10, varargin{:});
if ~use_cbf_filter
    %% Low-level controller maps desired force to input voltage.
    controller = @(t, x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        t, x, controller_for_force, varargin{:});
else
    controller_unfiltered = @(t, x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        t, x, controller_for_force, varargin{:});
    controller = @(t, x, varargin) dynsys.ctrlCbfQp(t, x, ...
        'u_ref', controller_unfiltered, varargin{:});
end

if introduce_delay
    params.tau_delay = 0.2;
    plant = QuanserCartPoleInputDelay(params, '2WEIGHTS');
else
    plant = QuanserCartPole(params, '2WEIGHTS');
end

T = 20;
% x0 = [10*pi/12; 0; 0; 0];
% x0 = [pi/12; 0; 0; 0];
% x0 = [0; pi-0.01; 0; -0.01];

x0 = [0; pi-0.01; 0; -0.01];
% x0 = [0; pi/24; 0; 0];
% x0 = [-0.34; -0.19; -0.5; 2.17];

if introduce_delay
    x0 = [x0; 0.0];
end
[xs, us, ts, extraout] = rollout_controller(x0, plant, controller, T);
if isfield(extraout, 'swing_up')
    swing_up_flags = cell2mat(extraout.swing_up);
end
if isfield(extraout, 'feas')
    feas = extraout.feas;
end

% Evaluate linear momentum and total energy
Ps = zeros(size(ts));
Es = zeros(size(ts));
Eps = zeros(size(ts));
Bs = zeros(size(ts));
Vs = zeros(size(ts));
cbf_constraints = zeros(size(ts));
for i = 1:length(ts)
    Ps(i) = dynsys.linear_momentum(xs(:, i));
    Es(i) = dynsys.total_energy(xs(:, i));
    Eps(i) = dynsys.pole_energy(xs(:, i));
    Bs(i) = dynsys.cbf(xs(:, i));
    Vs(i) = dynsys.clf(xs(:, i));
    cbf_constraints = dynsys.dcbf(xs(:, i), us(:, i)) + dynsys.cbf_rate * dynsys.cbf(xs(:, i));
end

fig = open_figure('font_size', 15, 'size', [1200, 900]);

if exist('swing_up_flags', 'var')
    num_plots = 6;
else
    num_plots = 5;
end

subplot(num_plots, 1, 1);
plot(ts, xs(1, :));
hold on;
line([ts(1), ts(end)], params.x_lim * ones(1, 2), 'Color', 'r');
line([ts(1), ts(end)], -params.x_lim * ones(1, 2), 'Color', 'r');
ylabel('$x_1$ ($s$)');
grid on;

subplot(num_plots, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$ ($\theta$)');
grid on;

subplot(num_plots, 1, 3);
plot(ts, xs(3, :));
ylabel('$x_3$ ($\dot{s}$)');
grid on;

subplot(num_plots, 1, 4);
plot(ts, xs(4, :));
ylabel('$x_4$ ($\dot{\theta}$)');
grid on;

subplot(num_plots, 1, 5);
plot(ts, us);
ylabel('$u$');
if introduce_delay
    hold on;
    plot(ts, xs(5, :));
    us_delayed_estimate = estimate_delayed_input_signal(us, ts, params.tau_delay);
    plot(ts, us_delayed_estimate);
    legend('$u$', '$u_{true}$', '$u_{shifted}$', 'interpreter', 'latex');
end    
grid on;

if exist('swing_up_flags', 'var')
    subplot(num_plots, 1, 6);
    scatter(ts, swing_up_flags);
    ylabel('Swing up mode');
    grid on;
end
xlabel('$t$');


fig2 = open_figure('font_size', 15, 'size', [1200, 900]);
subplot(3, 1, 1);
plot(ts, Ps);
ylabel('$P(t)$ (Linear Momentum)');
grid on;

subplot(3, 1, 2);
plot(ts, Es);
ylabel('$E(t)$ (Total Energy)');
xlabel('$t$');
grid on;

subplot(3, 1, 3);
plot(ts, Eps);
hold on;
line([ts(1), ts(end)], dynsys.potential_energy_upright() * ones(1, 2), 'Color', 'r');
ylabel('$E_p(t)$ (Pendulum Energy)');
xlabel('$t$');
grid on;


fig = plot_cart_pole_core_results(xs, us, ts, dynsys);

% save_figure(fig, 'file_name', 'swing_up_without_cbf_sim', 'file_format', 'pdf', 'figure_size', [12, 18]);
% save_figure(fig, 'file_name', 'swing_up_with_cbf_sim', 'file_format', 'pdf', 'figure_size', [12, 18]);
% fig3 = dynsys.animate_cart_pole(xs, us, ts, 0.01, 'zero');


