clear all
close all

K_mimo = cell(2, 1);
K_mimo{1} = [0, 1];
K_mimo{2} = [1, 2];
params.K_mimo = K_mimo;
params.rel_deg_y = [2; 2];
plant = AirCraftCtol(params);

% params.epsilon = 0;
K_mimo = cell(2, 1);
K_mimo{1} = [0, 1];
K_mimo{2} = [1, 3, 3];
params.K_mimo = K_mimo;
params.rel_deg_y = [2; 3];
approx_dyn_sys = AirCraftCtol(params);

% v_trim = sqrt(1/plant.aL);
v_trim = 0.17;
ref_altitude = 0;

virtual_input_controller = @(t, x, varargin) plant.ctrlMimoTracking( ...
    t, x, @(t) get_trim_traj(t, v_trim, ref_altitude), varargin{:});
fl_controller = @(t, x, varargin) plant.ctrlFeedbackLinearize( ...
    t, x, virtual_input_controller, varargin{:});

virtual_input_controller2 = @(t, x, varargin) approx_dyn_sys.ctrlMimoTracking( ...
    t, x, @(t) get_trim_traj_extended(t, v_trim, ref_altitude), varargin{:});
approx_fl_controller = @(t, x, varargin) approx_dyn_sys.ctrlFeedbackLinearize( ...
    t, x, virtual_input_controller2, varargin{:});


x0 = [0; v_trim; 0; 0; 0; 0];
t_sim = 4;
[xs, us, ts, extraout] = rollout_controller(x0, plant, approx_fl_controller, t_sim);
ys = extraout.ys;
% ys_d = cell2mat(extraout.y_d);
% xi_ds = cell2mat(extraout.xi_d);
open_figure('font_size', 18, 'size', [1000, 1200]);

subplot(6, 1, 1);
plot(ts, xs(1, :), 'LineWidth', 1.5);
ylabel('$x$');
grid on;

subplot(6, 1, 2);
plot(ts, xs(2, :), 'LineWidth', 1.5);
ylabel('$\dot{x}$');
grid on;

subplot(6, 1, 3);
plot(ts, xs(3, :), 'LineWidth', 1.5);
% hold on;
% plot(ts, ys_d(2, :), 'r:', 'LineWidth', 1.5);
ylabel('$z$');
grid on;

subplot(6, 1, 4);
plot(ts, xs(4, :), 'LineWidth', 1.5);
ylabel('$\dot{z}$');
grid on;

subplot(6, 1, 5);
plot(ts, xs(5, :), 'LineWidth', 1.5);
ylabel('$\theta$');
grid on;

subplot(6, 1, 6);
plot(ts, xs(6, :), 'LineWidth', 1.5);
ylabel('$\dot{\theta}$');
xlabel('$t$');
grid on;



function y_d = get_trim_traj(t, v_trim, ref_altitude)
    v = v_trim;
    z = ref_altitude;
    y_d{1} = [v * t; v;0;];
    y_d{2} = [z; 0; 0];
end

function y_d = get_trim_traj_extended(t, v_trim, ref_altitude)
    v = v_trim;
    z = ref_altitude;
    y_d{1} = [v * t; v;0;];
    y_d{2} = [z; 0; 0; 0];
end

