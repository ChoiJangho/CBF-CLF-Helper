clear all;
% close all;
dt = 0.01;

% Type options:
%   'ONES': set every model parameters to 1 (including the gravity)
%   'QUANSER': Using the full quanser parameter values
%   'QUANSER_NO_DRAG': Using the full quanser parameter values except for
%   the drag terms.
%   'QUANSER_SIMPLE': Use only the mass and the length values from Quanser.
params = get_predefined_params_set_for_vanilla_cart_pole('QUANSER');

% Set up input saturation limit.
params.u_max = (params.m + params. M) * 6;
% Set up CLF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;
% Create the dynamic system to simulate.
dynsys = CartPole(params);

%% Choice of controllers
%% zero control input
% controller = @(t, x, varargin) dynsys.ctrl_zero(t, x, varargin{:});

%% locally stabilizing controllers. (Sontag controller works very well)
% controller = @dynsys.ctrlClfSontag;
% controller = @dynsys.ctrlClfQp;

%% swing-up controllers (only the last one switches the control mode near the origin.)
% controller = @(t, x, varargin) dynsys.ctrl_pump_energy(t, x, varargin{:});
% controller = @(t, x, varargin) dynsys.ctrl_pump_energy(t, x, 'k_energy', 50, varargin{:});
controller = @(t, x, varargin) dynsys.ctrl_hybrid_swing_up(t, x, 'k_energy', 5, varargin{:});

% Simulation time.
T = 20;
% Initial state
% x0 = [0; pi/2; 0; 0];
% x0 = [0; pi/12; 0; 0];
x0 = [0; pi; 0; 0]; % released position.
% x0 = [-0.42; 0.28; -0.135; -3.87];

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, controller, T);
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
for i = 1:length(ts)
    Ps(i) = dynsys.linear_momentum(xs(:, i));
    Es(i) = dynsys.total_energy(xs(:, i));
    Eps(i) = dynsys.pole_energy(xs(:, i));
end

fig = open_figure('font_size', 15, 'size', [1200, 900]);

if exist('swing_up_flags', 'var')
    num_plots = 6;
else
    num_plots = 5;
end

subplot(num_plots, 1, 1);
plot(ts, xs(1, :));
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
grid on;

if exist('swing_up_flags', 'var')
    subplot(num_plots, 1, 6);
    scatter(ts, swing_up_flags);
    ylabel('Swing up mode');
    xlabel('$t$');
    grid on;
end

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

