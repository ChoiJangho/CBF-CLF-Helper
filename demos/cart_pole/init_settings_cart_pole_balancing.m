clear all;
close all;
dt = 0.01;

params.l = 9.81;    % [m]        length of pendulum
params.m = 20;    % [kg]       mass of pendulum
params.M = 25;    % [kg]       mass of cart
params.g = 9.81;   % [m/s^2]    acceleration of gravity
params.clf.rate = 0;

output_to_regulate = 's';
feedback_gain = [2, 3];

dynsys = CartPoleSimple(params, output_to_regulate, feedback_gain);

simple_fl_controller = @(x, varargin) dynsys.ctrlFeedbackLinearize( ...
    x, @dynsys.ctrlSisoLinearFeedback, varargin{:});

x0 = [pi/12; 0; 0; 0];

[xs, us, ts, extraout] = rollout_controller(x0, dynsys, dynsys, simple_fl_controller, 5);

figure;
subplot(5, 1, 1);
plot(ts, xs(1, :));
ylabel('x_1');
subplot(5, 1, 2);
plot(ts, xs(2, :));
ylabel('x_2');
subplot(5, 1, 3);
plot(ts, xs(3, :));
ylabel('x_3');
subplot(5, 1, 4);
plot(ts, xs(4, :));
ylabel('x_4');

subplot(5, 1, 5);
plot(ts, us);
ylabel('u');
