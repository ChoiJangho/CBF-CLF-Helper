clear all;
% close all;
dt = 0.01;

% params.l = 1;    % [m]        length of pendulum
% params.m = 1;    % [kg]       mass of pendulum
% params.M = 1;    % [kg]       mass of cart
% params.g = 1;   % [m/s^2]    acceleration of gravity
params.l = 0.0254 * 25.25;    % [m]        length of pendulum
params.m = 0.23;    % [kg]       mass of pendulum
params.M = 0.57;    % [kg]       mass of cart
params.gravity = 9.81;   % [m/s^2]    acceleration of gravity
% params.J = 0;
params.clf.rate = 0;

dynsys = CartPole(params);

% zero control input
controller = @(x, varargin) dynsys.ctrl_zero([], x, varargin{:});
% feedback_linearization
% controller = @(x, varargin) dynsys.ctrlFeedbackLinearize( ...
%     x, @dynsys.ctrlSisoLinearFeedback, varargin{:});
% sontag_controller
% controller = @(x, varargin) dynsys.ctrlClfSontag(x, varargin{:});

T = 5;
% x0 = [10*pi/12; 0; 0; 0];
% x0 = [pi/12; 0; 0; 0];
% x0 = [0; deg2rad(90); 0; 0];
x0 = [0; pi/12; 0; 0];

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, dynsys, controller, T);

% Evaluate linear momentum and total energy
Ps = zeros(size(ts));
Es = zeros(size(ts));
for i = 1:length(ts)
    Ps(i) = dynsys.linear_momentum(xs(:, i));
    Es(i) = dynsys.total_energy(xs(:, i));
end

fig = open_figure('font_size', 15, 'size', [1200, 900]);
subplot(5, 1, 1);
plot(ts, xs(1, :));
ylabel('$x_1$ ($s$)');
grid on;

subplot(5, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$ ($\theta$)');
grid on;

subplot(5, 1, 3);
plot(ts, xs(3, :));
ylabel('$x_3$ ($\dot{s}$)');
grid on;

subplot(5, 1, 4);
plot(ts, xs(4, :));
ylabel('$x_4$ ($\dot{\theta}$)');
grid on;

subplot(5, 1, 5);
plot(ts, us);
ylabel('$u$');
xlabel('$t$');
grid on;

fig2 = open_figure('font_size', 15, 'size', [1200, 900]);
subplot(2, 1, 1);
plot(ts, Ps);
ylabel('$P(t)$ (Linear Momentum)');
grid on;

subplot(2, 1, 2);
plot(ts, Es);
ylabel('$E(t)$ (Total Energy)');
xlabel('$t$');
grid on;

% fig3 = dynsys.animate_cart_pole(xs, us, ts, 0.01, 'zero');


