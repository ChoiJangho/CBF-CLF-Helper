%% Example for CBF and CLF simulation for cartpole
clear all;
close all;
dt = 0.002;
sim_t = 10;
x0 = [0;deg2rad(12);0;0];

params.IP02_LOAD_TYPE = 'WEIGHT';
params.PEND_TYPE = 'LONG_24IN';
params.u_max = 10; % max voltage
params.u_min = -10; % min voltage
params.x_lim = 0.3;
params.weight_slack = 1e3;
params.k_cbf = 10;

params.clf.rate = 1;
params.cbf.rate = 10;

cartPole = QuanserCartPole(params);

controller = @(x, varargin) cartPole.ctrlCbfClfQp(x, ...
    'weight_slack', params.weight_slack, varargin{:});

[xs, us, ts, extraout] = rollout_controller( ...
    x0, cartPole, cartPole, controller, sim_t, 'dt', dt);
Bs = extraout.Bs;
Vs = extraout.Vs;
slacks = extraout.slacks;
plot_results(ts, xs, us, Bs, Vs, slacks)

function plot_results(t, xs, us, Bs, Vs, slacks)

figure
subplot(4,1,1)
plot(t, xs(1, :))
xlabel('t')
ylabel('x [m]')

subplot(4,1,2)
plot(t, xs(2, :))
xlabel('t')
ylabel('theta [rad]')

subplot(4,1,3)
plot(t, xs(3, :))
xlabel('t')
ylabel('dx [m/sec]')

subplot(4,1,4)
plot(t, xs(4, :))
xlabel('t')
ylabel('dtheta [rad/sec]')

figure
subplot(3,1,1)
plot(t, Bs)
xlabel('t')
ylabel('B')

subplot(3,1,2)
plot(t, Vs)
xlabel('t')
ylabel('V')

subplot(3,1,3)
plot(t, slacks)
xlabel('t')
ylabel('slack')


figure
plot(t, us)
xlabel('t')
ylabel('u [V]')

end