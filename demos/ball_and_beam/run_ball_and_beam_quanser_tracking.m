clear all
close all

% plot animation if true.
plot_animation = false;
% save animation to video if true.
save_video = true;
period = 10;
amplitude = 0.1;

dt = 0.001;
eps = 1.0;
s0 = 3.0;
k_p = s0^4/eps^4;
k_d = 4*s0^3/eps^3;
k_a = 6*s0^2/eps^2;
k_j = 4*s0/eps;
feedback_gain = [k_p, k_d, k_a, k_j];

dynsys = BallBeamQuanser(feedback_gain);


virtual_input_controller = @(t, x, varargin) dynsys.ctrlSisoTracking( ...
    t, x, @(t) get_reference_trajectory(t, amplitude, period, 'sine'), varargin{:});

fl_controller = @(t, x, varargin) dynsys.ctrlFeedbackLinearize( ...
    t, x, virtual_input_controller, varargin{:});
pd_controller = @(x, varargin) dynsys.ctrlPD(x, varargin{:});

x0 = [-0.19; 0; 0; 0];

t_sim = 20;
[xs, us, ts, extraout] = rollout_time_varying_controller(x0, dynsys, dynsys, fl_controller, ...
    t_sim, 'end_event_function', @(t, x) dynsys.ball_out_of_range(t, x));
xis = cell2mat(extraout.xi);
xi_ds = cell2mat(extraout.xi_d);
ref_ps = xi_ds(1, :);
ref_vs = xi_ds(2, :);

open_figure('font_size', 18, 'size', [1000, 1200]);
subplot(5, 1, 1);
plot(ts, 100 * xs(1, :), 'LineWidth', 1.5);
hold on;
plot(ts, 100 * ref_ps, 'r:', 'LineWidth', 1.5);
ylabel('$z_{ball}$ [cm]');
grid on;

subplot(5, 1, 2);
plot(ts, 100 * xs(2, :), 'LineWidth', 1.5);
hold on;
plot(ts, 100 * ref_vs, 'r:', 'LineWidth', 1.5);
ylabel('$\dot{z}_{ball}$ [cm / s]');
grid on;

subplot(5, 1, 3);
plot(ts, 180 * xs(3, :) / pi, 'LineWidth', 1.5);
ylabel('$\theta$ [deg]');
grid on;

subplot(5, 1, 4);
plot(ts, 180 * xs(4, :) / pi, 'LineWidth', 1.5);
ylabel('$\dot{\theta}$ [deg/s]');
grid on;

subplot(5, 1, 5);
plot(ts, us, 'LineWidth', 1.5);
ylabel('$u$ [V]');
xlabel('$t$ [sec]');
grid on; 


open_figure('font_size', 18);
subplot(4, 1, 1);
plot(ts, xis(1, :), 'LineWidth', 1.5); hold on;
plot(ts, xi_ds(1, :), 'r:', 'LineWidth', 1.5);
ylabel('$y$');

subplot(4, 1, 2);
plot(ts, xis(2, :), 'LineWidth', 1.5);
hold on;
plot(ts, xi_ds(2, :), 'r:', 'LineWidth', 1.5);
ylabel('$\dot{y}$');

subplot(4, 1, 3);
plot(ts, xis(3, :), 'LineWidth', 1.5);
hold on;
plot(ts, xi_ds(3, :), 'r:', 'LineWidth', 1.5);
ylabel('$\ddot{y}$');

subplot(4, 1, 4);
plot(ts, xis(4, :), 'LineWidth', 1.5);
hold on;
plot(ts, xi_ds(4, :), 'r:', 'LineWidth', 1.5);
ylabel('$\ddot{y}$');
xlabel('$t$ [sec]');
grid on;


if plot_animation
    animate_ball_and_beam_quanser(ts, xs, ref_ps, save_video);
end

