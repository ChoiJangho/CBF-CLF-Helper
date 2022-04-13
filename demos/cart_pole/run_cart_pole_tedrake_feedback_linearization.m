clear all;
close all;
dt = 0.01;

params.l = 1;    % [m]        length of pendulum
params.m = 1;    % [kg]       mass of pendulum
params.M = 1;    % [kg]       mass of cart
params.g = 1;   % [m/s^2]    acceleration of gravity
params.J = 0;
params.clf.rate = 0;

output_to_regulate = 'theta';
feedback_gain = [9, 6];

dynsys = CartPoleSimple(params, output_to_regulate, feedback_gain);

simple_fl_controller = @(x, varargin) dynsys.ctrlFeedbackLinearize( ...
    x, @dynsys.ctrlSisoLinearFeedback, varargin{:});
sontag_controller = @(x, varargin) dynsys.ctrlClfSontag(x, varargin{:});

% x0 = [10*pi/12; 0; 0; 0];
% x0 = [pi/12; 0; 0; 0];
% x0 = [pi/12; 0; 0; 0];
x0 = [pi/3; 0; 0; 0];
% x0 = [0; 0.5; 0; 0];


%% Feedback Linearization
[xs, us, ts, extraout] = rollout_controller(x0, dynsys, dynsys, simple_fl_controller, 20);
zs = cell2mat(extraout.z);
%% Sontag controller
% [xs, us, ts, extraout] = rollout_controller(x0, dynsys, dynsys, sontag_controller, 20);
% zs  = [];


% [xs, us, ts, extraout] = rollout_controller(x0, dynsys, dynsys, ...
%    @(x, varargin) ctrl_zero(x, varargin{:}), 5);


fig = open_figure('font_size', 15, 'size', [1200, 900]);
subplot(5, 1, 1);
plot(ts, xs(1, :));
ylabel('$x_1$ ($\theta$)');
subplot(5, 1, 2);
plot(ts, xs(2, :));
ylabel('$x_2$ ($s$)');
subplot(5, 1, 3);
plot(ts, xs(3, :));
ylabel('$x_3$ ($\dot{\theta}$)');
subplot(5, 1, 4);
plot(ts, xs(4, :));
ylabel('$x_4$ ($\dot{s}$)');

subplot(5, 1, 5);
plot(ts, us);
ylabel('$u$');
xlabel('$t$');


save_figure('file_name', 'exercise_3_2_result', 'figure_size', [9, 12], 'file_format', 'png');

fig = open_figure('font_size', 15, 'size', [1000, 1000]);
plot(zs(1, :), zs(2, :), 'LineWidth', 2);
if strcmp(output_to_regulate, 'theta')
    xlabel('$s$');
else
    xlabel('$\theta$');
end
ylabel('$\dot{s}\cos{\theta}-\dot{\theta}$');

save_figure('file_name', 'exercise_3_2_internal_dynamics', 'figure_size', [9, 6], 'file_format', 'png');


% fig = dynsys.draw_rollout(xs, us, ts, 0.01, 'zero');

function [u, extraout] = ctrl_zero(x, varargin)
    u = 0;
    extraout = [];
end



