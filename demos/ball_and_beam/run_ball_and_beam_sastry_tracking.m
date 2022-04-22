clear all
close all

% plot animation if true.
plot_animation = false;
% save animation to video if true.
save_video = true;
% reference trajectory settings.
period = 40;
amplitude = 10;

dt = 0.001;
s0 = 1.0;
k_p = s0^4;
k_d = 4*s0^3;
k_a = 6*s0^2;
k_j = 4*s0;
feedback_gain = [k_p, k_d, k_a, k_j];
params.K_siso = feedback_gain;

model = BallBeamSastryApprox1(params);
% plant = BallBeamSastryApprox1(params);
plant = BallBeamSastry(params);

virtual_input_controller = @(t, x, varargin) model.ctrlSisoTracking( ...
    t, x, @(t) get_reference_trajectory(t, amplitude, period, 'sine'), varargin{:});

fl_controller = @(t, x, varargin) model.ctrlFeedbackLinearize( ...
    t, x, virtual_input_controller, varargin{:});
pd_controller = @(x, varargin) model.ctrlPD(x, varargin{:});

x0 = [0; 0; 0; 0];

t_sim = 40;
[xs, us, ts, extraout] = rollout_time_varying_controller(x0, plant, model, fl_controller, ...
    t_sim, 'end_event_function', @(t, x) plant.ball_out_of_range(t, x));
xis = cell2mat(extraout.xi);
xi_ds = cell2mat(extraout.xi_d);
ref_ps = xi_ds(1, :);
ref_vs = xi_ds(2, :);

% Reproducing figure 10.3
open_figure('font_size', 18, 'size', [1600, 1000]);
subplot(2, 2, 1);
plot(ts, ref_ps - xs(1, :), 'LineWidth', 1.5);
ylabel('$e=y_d - r$ [m]');
grid on;

subplot(2, 2, 2);
plot(ts, xs(1, :) .* xs(4, :).^2,  'LineWidth', 1.5);
ylabel('$\psi_2$');
grid on;

subplot(2, 2, 3);
plot(ts, xs(3, :), 'LineWidth', 1.5);
ylabel('$\theta$ [rad]');
grid on;

subplot(2, 2, 4);
plot(ts, xs(1, :), 'LineWidth', 1.5);
hold on;
plot(ts, ref_ps, 'r:', 'LineWidth', 1.5);
ylabel('$r$ [m]');
grid on;

open_figure('font_size', 18, 'size', [1000, 1200]);
subplot(5, 1, 1);
plot(ts, xs(1, :), 'LineWidth', 1.5);
hold on;
plot(ts, ref_ps, 'r:', 'LineWidth', 1.5);
ylabel('$r_{ball}$ [m]');
grid on;

subplot(5, 1, 2);
plot(ts, xs(2, :), 'LineWidth', 1.5);
hold on;
plot(ts, ref_vs, 'r:', 'LineWidth', 1.5);
ylabel('$\dot{r}_{ball}$ [m / s]');
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
plot(ts, 180 * us / pi, 'LineWidth', 1.5);
ylabel('$u [deg/s^2]$');
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


% if plot_animation
%     animate_ball_and_beam_sastry(ts, xs, ref_ps, save_video, 4);
% end

