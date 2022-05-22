%% Example for CBF and CLF simulation for cartpole
clear all;
close all;
dt = 0.01;
sim_t = 5;
x0 = [0;deg2rad(180);0;0]; % downright
% x0 = [0; 0; 0; 0]; % upright
x0 = [0; 0.01; 0; 0]; % upright


params.IP02_LOAD_TYPE = 'NO_LOAD';
params.PEND_TYPE = 'LONG_24IN';
params.u_max = 6; % max voltage
params.u_min = -6; % min voltage
params.x_lim = 0.3;
params.weight_slack = 1e3;
params.k_cbf = 10;

params.clf.rate = 1;
params.cbf.rate = 10;

cartPole = QuanserCartPole(params, 'symbolic', false);

k_swing_up = 200;
epsilon_switch = deg2rad(10);
controller_test = @(t, x, varargin) cartPole.ctrl_zero(t, x, varargin{:});
% controller_test = @(t, x, varargin) cartPole.ctrl_periodic_bang_bang(t, x, 4, varargin{:});
controller_raw = @(t, x, varargin) cartPole.ctrl_hybrid_swing_up(t, x, ...
    k_swing_up, epsilon_switch, varargin{:});
controller_safe = @(t, x) cartPole.ctrlCbfQp( t, x, ...
    'u_ref', @(t, x) cartPole.ctrl_hybrid_swing_up(t, x, k_swing_up));


[xs, us, ts, extraout] = rollout_time_varying_controller( ...
    x0, cartPole, cartPole, controller_test, sim_t, 'dt', dt);
plot_quanser_cart_pole_result(ts, xs, us, [], [], [])
ps = zeros(size(ts));
Es = zeros(size(ts));
for i = 1:length(ts)
    ps(i) = cartPole.get_linear_momentum(xs(:, i));
    Es(i) = cartPole.get_total_energy2(xs(:, i));
    
end
figure;
subplot(2, 1, 1);
plot(ts, ps);
subplot(2, 1, 2);
plot(ts, Es);

% cartPole.draw_rollout(xs, us, ts, 0.01, 'zero');

