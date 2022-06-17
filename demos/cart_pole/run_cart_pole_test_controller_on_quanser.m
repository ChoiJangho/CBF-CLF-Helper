clear all;
% close all;
dt = 0.01;

% Chocice of Model Type: options:
%   'ONES': set every model parameters to 1 (including the gravity)
%   'QUANSER': Using the full quanser parameter values
%   'QUANSER_NO_DRAG': Using the full quanser parameter values except for
%   the drag terms.
%   'QUANSER_SIMPLE': Use only the mass and the length values from Quanser.
params = get_predefined_parameter_set('QUANSER');

% Set up input saturation limit.
params.u_max = (params.m + params. M) * 10;
% Set up CLF-related parameters
params.clf.rate = 0.5;
params.weight_slack = 1e10;
% Create the dynamic system to simulate.
model_sys = CartPole(params);

params.k_cbf = 10;
params.x_lim = 0.3;
params.cbf.rate = 10;
params.u_max = 6;
params.u_min = -6;
dynsys = QuanserCartPole(params);

%% Choice of controllers
%% zero control input in voltage.
% controller = @(x, varargin) dynsys.ctrl_zero([], x, varargin{:});
%% zero control input in cart force.
% controller = @(x, varargin) dynsys.ctrl_zero_force([], x, varargin{:});

%% Choice of high-level controller
% controller_for_force = @(x, varargin) model_sys.ctrlClfSontag(x, varargin{:});
controller_for_force = @(x, varargin) model_sys.ctrl_hybrid_swing_up( ...
  [], x, 'k_energy', 10, varargin{:});

%% Applying safety filter to the swing-up controller
% controller_for_force = @(x, varargin) model_sys.ctrl_hybrid_swing_up( ...
%   [], x, 'k_energy', 10, varargin{:});
% controller_unfiltered = @(x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
%     [], x, controller_for_force, varargin{:});
% controller = @(x, varargin) dynsys.ctrlCbfQp(x, ...
%     'u_ref', controller_unfiltered, varargin{:});

%% Low-level controller maps desired force to input voltage.
controller = @(x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
    [], x, controller_for_force, varargin{:});


T = 20;
% x0 = [10*pi/12; 0; 0; 0];
% x0 = [pi/12; 0; 0; 0];
x0 = [0; pi-0.01; 0; -0.01];
% x0 = [0; pi/24; 0; 0];
% x0 = [-0.34; -0.19; -0.5; 2.17];

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, model_sys, controller, T);
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


% fig3 = dynsys.animate_cart_pole(xs, us, ts, 0.01, 'zero');


