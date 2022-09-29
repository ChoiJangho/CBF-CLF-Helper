%% This example demonstrates
% - the default usage of the CtrlAffineSys.
% - how to define function handle of a CBF-CLF-QP controller while passing additional arguments.
% close all;
clear all;

%% General
dt = 0.02;
max_acc = 100;
max_angular_acc = 100;
u_max = [max_acc; max_angular_acc];

%% % Control parameters
params.u_max = u_max;
params.u_min = -u_max;
params.clf.rate = 1.6;
params.weight.slack = 1000000;
params.v_target = 5;

%% CBF
params.xo = 20;
params.yo = 1;
params.Ro = 2;
params.gamma_l = 5;
params.cbf.rate = 5;

dynsys = KinematicVehicle(params);
% Additional options for the controller.
with_slack = 1; % activate slack explicitly.
% the reference control is set to minimize the difference between the next
% u and the previous u.
u_ref = 'min_ctrl_diff'; 
% ratio determines the cost portion of the difference in u compared to the
% norm of u.
ratio_ctrl_diff = 0.93;

% Define the function handle with additional input arguments that specify
% the controller settings.
controller = @(t, x, varargin) dynsys.ctrl_cbf_clf_qp(t, x, ...
    'with_slack', with_slack, 'u_ref', u_ref, 'ratio_ctrl_diff', ratio_ctrl_diff, varargin{:});
% controller = @(t, x, varargin) dynsys.ctrl_cbf_clf_qp(t, x, ...
%     'with_slack', with_slack, varargin{:});


verbose_level = 1;

%% Results
% initial state
x0 = [0;0.25;2.0;0.25;0.25];
% simulation time
sim_t = 20;
[xs, us, ts, extraout] = rollout_controller(x0, dynsys, controller, ...
    sim_t, 'dt', dt, 'verbose_level', verbose_level);

%% Plot state history
figure;
subplot(5,1,1)
plot(ts, xs(1, :)); hold on;
ylabel('x [m]')

subplot(5,1,2)
plot(ts, xs(2, :)); hold on;
ylabel('y [m]')

subplot(5,1,3)
ylabel('v [m/s]')
plot(ts, xs(3, :)); hold on;
subplot(5,1,4)
plot(ts, xs(4, :)); hold on;
ylabel('theta [rad]')

subplot(5,1,5)
plot(ts, xs(5, :)); hold on;
xlabel('t')
ylabel('sigma [rad/s]')

%% Plot control history
figure;
subplot(3, 1, 1);
plot(ts, us(1, :));
xlabel('t');
ylabel('u1');
subplot(3, 1, 2);
plot(ts, us(2, :));
xlabel('t');
ylabel('u2');
subplot(3, 1, 3);
plot(ts, extraout.slacks);
xlabel('t');
ylabel('slack');

%% Plot Trajectory with the obstacle.
figure;
p = plot(xs(1, :), xs(2, :)); hold on; grid on;
draw_circle([params.xo; params.yo], params.Ro);
xlabel('$x$ [m]', 'Interpreter', 'latex')
ylabel('$y$ [m]', 'Interpreter', 'latex')
axis equal;

function h = draw_circle(center,r)
hold on
th = 0:pi/50:2*pi;
xunit = r * cos(th) + center(1);
yunit = r * sin(th) + center(2);
h = plot(xunit, yunit);
hold off
end
