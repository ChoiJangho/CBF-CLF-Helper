clear all;
close all;
dt = 0.01;

params.l = 9.81;    % [m]        length of pendulum
params.m = 20;    % [kg]       mass of pendulum
params.M = 25;    % [kg]       mass of cart
params.gravity = 9.81;   % [m/s^2]    acceleration of gravity
params.clf.rate = 0; % not used in this demo.

output_to_regulate = 'theta';
feedback_gain = [2, 3];

dynsys = CartPoleFl(params, output_to_regulate, feedback_gain);

simple_fl_controller = @(t, x, varargin) dynsys.ctrlFeedbackLinearize( ...
    t, x, @dynsys.ctrlSisoLinearFeedback, varargin{:});

x0 = [0; pi/12; 0; 0];

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, simple_fl_controller, 5);
zs = cell2mat(extraout.z);

fig = open_figure('font_size', 15, 'size', [1200, 900]);
subplot(5, 1, 1);
plot(ts, xs(1, :));
ylabel('$x_1$ ($\theta$)');
grid on;
subplot(5, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$ ($s$)');
grid on;

subplot(5, 1, 3);
plot(ts, xs(3, :));
ylabel('$x_3$ ($\dot{\theta}$)');
grid on;

subplot(5, 1, 4);
plot(ts, xs(4, :));
ylabel('$x_4$ ($\dot{s}$)');
grid on;

subplot(5, 1, 5);
plot(ts, us);
ylabel('$u$');
xlabel('$t$');
grid on;


fig = open_figure('font_size', 15, 'size', [1000, 1000]);
plot(zs(1, :), zs(2, :), 'LineWidth', 2);
if strcmp(output_to_regulate, 'theta')
    xlabel('$s$');
else
    xlabel('$\theta$');
end
ylabel('$\dot{s}\cos{\theta}-\dot{\theta}$');

