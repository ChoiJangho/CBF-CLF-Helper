%% This example demonstrates
% - Target pursuing navigation of a simple 4D nonholonomic vehicle while avoiding obstacle.
% - the usafe of a time-varying reference controller and the CBF-QP as the safety filter.
close all;
clear all;

%% General
dt = 0.02;
max_acc = 1;
max_yaw_rate = 0.5;
u_max = [max_yaw_rate; max_acc];

%% % Control parameters
params.u_max = u_max;
params.u_min = -u_max;
params.weight.slack = 1000000;
params.v_target = 5;
params.v_max = 10;
params.max_acc = max_acc;

%% CBF
params.xo = 0;
params.yo = 0;
params.Ro = 2;
params.gamma_l = 5;
params.cbf.rate = 1;

dynsys = Car4D(params);
% Additional options for the controller.
with_slack = 1; % activate slack explicitly.
% Define the function handle with additional input arguments that specify the controller settings.

% The reference controller is a target tracking controller. The target
% points alternates between (2, 2), (2, -2), (-2, -2), (-2, 2) for every 10 second (specified by 'time_per_target').
ref_controller = @(t, x, varargin) dynsys.ctrl_pursue_target(t, x, 'target', ...
    [2, 2; 2, -2; -2, -2; -2, 2]', 'v_ref', 1, 'time_per_target', 10, 'periodic', true, varargin{:});

% controller = ref_controller;
controller = @(t, x, varargin) dynsys.ctrl_cbf_qp(t, x, ...
   'with_slack', with_slack, 'u_ref', ref_controller, varargin{:});


verbose_level = 1;

%% Results
% initial state
% x0 = [-8;0.01;0.0;1.5];
% x0 = [-5;0;0.0;1];
% x0 = [-1;-2;0.0;1];
x0 = [4;0;0.0;1];

% simulation time
sim_t = 40;
[xs, us, ts, extraout] = rollout_controller(x0, dynsys, controller, ...
    sim_t, 'dt', dt, 'verbose_level', verbose_level);

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
draw_circle([params.xo; params.yo], params.Ro);
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
subplot(3, 1, 1);
plot(ts, extraout.Bs);
ylabel('B (CBF)');
subplot(3, 1, 2);
plot(ts, extraout.feas);
ylabel('feas');
xlabel('t');
subplot(3, 1, 3);
plot(ts, extraout.slacks);
xlabel('t');
ylabel('slack');


function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off
end
