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

%% Controller setting to test.
params.state_dependent_input_bound = true;
params.apply_cbf_smooth_margin = false;
active_input_bound = true;
with_slack = 0; % activate slack explicitly if 1.

%% % Control parameters
params.u_max = u_max;
params.u_min = -u_max;
params.weight.slack = 1000000;
params.v_target = 5;
params.v_max = 10;
%% CBF
params.xo = 0;
params.yo = 0;
params.Ro = 2;
params.gamma_l = 5;
params.cbf.rate = 1;

%% Grid
grid_min = [-10; -10; -pi; -0.5];  % Lower corner of computation domain
grid_max = [10; 10; pi; 2.5];      % Upper corner of computation domain
%N = [20; 20; 20; 20];           % Number of grid points per dimension
% N = [40; 40; 40; 40];
N = [101; 101; 60 ; 31];
% N = [21; 21; 10; 16];
g = createGrid(grid_min, grid_max, N, 3); % Create grid
data0 = Car4DGridCbf.get_target_function(g, params);

dynsys = Car4DGridCbf(params);
dynsys.init_cbf_tables(g, data0);
% Define the function handle with additional input arguments that specify the controller settings.

% The reference controller is a target tracking controller. The target
% points alternates between (2, 2), (2, -2), (-2, -2), (-2, 2) for every 10 second (specified by 'time_per_target').
ref_controller = @(t, x, varargin) dynsys.ctrl_pursue_target(t, x, 'target', ...
    [2, 2; 2, -2; -2, -2; -2, 2]', 'v_ref', 1, 'time_per_target', 10, 'periodic', true, varargin{:});

% controller = ref_controller;
controller = @(t, x, varargin) dynsys.ctrl_cbf_qp(t, x, ...
   'with_slack', with_slack, 'u_ref', ref_controller, 'active_input_bound', active_input_bound, varargin{:});


verbose_level = 1;

%% Results
% initial state
% x0 = [-8;0.01;0.0;1.5];
% x0 = [-5;0;0.0;1];
% x0 = [-1;-2;0.0;1];
x0 = [4;0;0.0;1];
% x0 = [1.8673; 0.7147; 
% simulation time
sim_t = 20;
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
