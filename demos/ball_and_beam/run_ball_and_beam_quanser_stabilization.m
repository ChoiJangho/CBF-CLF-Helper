clear all
close all

dt = 0.001;
eps = 1.0;
s0 = 3.0;
k_p = s0^4/eps^4;
k_d = 4*s0^3/eps^3;
k_a = 6*s0^2/eps^2;
k_j = 4*s0/eps;
feedback_gain = [k_p, k_d, k_a, k_j];

dynsys = BallBeamQuanser(feedback_gain);

%% Choose your favorite controller, comment out the other.
controller = @(t, x, varargin) dynsys.ctrlFeedbackLinearize( ...
    t, x, @dynsys.ctrlSisoLinearFeedback, varargin{:});
% controller = @(t, x, varargin) dynsys.ctrlPD(t, x, varargin{:});

x0 = [-0.19; 0; 0; 0];

t_sim = 10;
[xs, us, ts, extraout] = rollout_controller(x0, dynsys, controller, ...
    t_sim, 'end_event_function', @(t, x) dynsys.ball_out_of_range(t, x));

open_figure();
subplot(5, 1, 1);
plot(ts, 100 * xs(1, :), 'LineWidth', 1.5);
hold on;
ylabel('$z_{ball}$ [cm]');
grid on;

subplot(5, 1, 2);
plot(ts, 100 * xs(2, :), 'LineWidth', 1.5);
hold on;
ylabel('$\dot{z}_{ball}$ [cm / s]');

subplot(5, 1, 3);
plot(ts, 180 * xs(3, :) / pi, 'LineWidth', 1.5);
ylabel('$\theta$ [deg]');

subplot(5, 1, 4);
plot(ts, 180 * xs(4, :) / pi, 'LineWidth', 1.5);
ylabel('$\dot{\theta}$ [deg/s]');

subplot(5, 1, 5);
plot(ts, us, 'LineWidth', 1.5);
ylabel('$u$ [V]');
xlabel('$t$');
grid on; 

% animate_ball_and_beam_quanser(ts, xs, zeros(size(ts)))


