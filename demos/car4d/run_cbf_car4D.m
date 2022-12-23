%% This example demonstrates
% - Target pursuing navigation of a simple 4D nonholonomic vehicle while avoiding obstacle.
% - the usafe of a time-varying reference controller and the CBF-QP as the safety filter.
clear all; close all;

%% General
dt = 0.01;
max_acc = 1;
max_yaw_rate = 2.0;
u_max = [max_yaw_rate; max_acc];

%% Controller setting to test.
% if true, apply state dependent input bound that ensures v don't exceed [0, v_max].
params.state_dependent_input_bound = true; 
% if true, apply some smooth margin to the cbf level set so that the cbf captures the constraint v \in [0, v_max].
% therefore, setting this to true is useful only when state_dependent_input_bound=false.
params.apply_cbf_smooth_margin = false;
params.dt_for_cbf_guarantee = dt;
active_input_bound = true; % if false, apply no input bounds.
with_slack = 0; % activate slack explicitly if 1.

%% % Control parameters
params.u_max = u_max;
params.u_min = -u_max;
params.weight.slack = 1000000;
params.v_target = 3;
params.v_max = 5;
params.v_min = 0;
params.v_min_desired = 1;
%% CBF
params.xo = 0;
params.yo = 0;
params.Ro = 3;
params.cbf.rate = 10;

dynsys = Car4D(params);
dynsys_for_filter = Car4DForFilteredInput(params);

%% Define controllers
% The reference controller is a target tracking controller. The target
% points alternates between (2, 2), (2, -2), (-2, -2), (-2, 2) for every 10 second (specified by 'time_per_target').

% targets = [2, 2; 2, -2; -2, -2; -2, 2]';
% targets = [4, 4; 4, -4; -4, -4; -4, 4]';
% targets = 7 * [1, 1; 1, -1; -1, -1; -1, 1]'; % clockwise
% targets = 7 * [1, 1; -1, 1; -1, -1; 1, -1]'; % counterclockwise
% targets = 5 * [1, 1; -1, -1; -1, 1; 1, -1]'; % zigzag
targets = 7 * [1, 1; 2, 2;]'; % zigzag

time_per_target = 5;
raw_controller = @(t, x, varargin) dynsys.ctrl_pursue_target(t, x, 'target', ...
    targets, 'v_ref', params.v_target, 'time_per_target', time_per_target, 'periodic', true, varargin{:});
ref_controller = @(t, x, varargin) dynsys_for_filter.ctrl_lpf(t, x, raw_controller, varargin{:});

controller = ref_controller;
controller = @(t, x, varargin) dynsys.ctrl_cbf_qp(t, x, ...
'with_slack', with_slack, 'u_ref', ref_controller, 'active_input_bound', active_input_bound, varargin{:});

verbose_level = 1;

%% Results
show_animation = true;
% initial state
x0 = [4;0;0.0;5];
% simulation time
sim_t = 10;
[xs, us, ts, extraout] = rollout_controller(x0, dynsys, controller, ...
    sim_t, 'dt', dt, 'verbose_level', verbose_level);

LgBs = [];
for i  = 1:length(ts)
    LgB_i = dynsys.lg_cbf(xs(:, i));
    LgBs = [LgBs; LgB_i];
end

%% Plot state history
figure;
subplot(4,1,1)
plot(ts, xs(1, :)); hold on;
ylabel('x [m]')

subplot(4,1,2)
plot(ts, xs(2, :)); hold on;
ylabel('y [m]')

subplot(4,1,3)
plot(ts, xs(3, :)); hold on;
ylabel('theta [rad]')

subplot(4,1,4)
plot(ts, xs(4, :)); hold on;
ylabel('v [m/s]')
xlabel('t')

%% Plot Trajectory with the obstacle.
figure;
p = plot(xs(1, :), xs(2, :)); hold on; grid on;
draw_circle([params.xo; params.yo], params.Ro, 'face_alpha', 0.3); hold on;
scatter(targets(1, :), targets(2, :))

xlabel('$x$ [m]', 'Interpreter', 'latex')
ylabel('$y$ [m]', 'Interpreter', 'latex')
axis equal;

%% Plot control history
figure;
subplot(2, 1, 1);
plot(ts, us(1, :));
xlabel('t');
ylabel('omega');
subplot(2, 1, 2);
plot(ts, us(2, :));
xlabel('t');
ylabel('a');

figure;
if isfield(extraout, 'slacks')
    n_plot = 3;
else
    n_plot = 2;
end
subplot(n_plot, 1, 1);
plot(ts, extraout.Bs);
ylabel('B (CBF)');
fprintf("Minimum of B(x): %.3f\n", min(extraout.Bs));
subplot(n_plot, 1, 2);
plot(ts, extraout.feas);
ylabel('feas');
xlabel('t');
if isfield(extraout, 'slacks')
subplot(n_plot, 1, 3);
plot(ts, extraout.slacks);
xlabel('t');
ylabel('slack');
end

figure;
subplot(2, 1, 1);
plot(ts, LgBs(:, 1));
subplot(2, 1, 2);
plot(ts, LgBs(:, 2));


%% Animation of car 4d
if show_animation
obstacle.center = [dynsys.params.xo; dynsys.params.yo];
obstacle.radius = dynsys.params.Ro;
animate_car4d_trajectory(ts, xs, 'obstacle', obstacle);
end