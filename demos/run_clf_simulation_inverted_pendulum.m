clear all; close all;

dt = 0.02;
sim_t = 5;

%% Model Parameters
params.l = 1;    % [m]        length of pendulum
params.m = 1;    % [kg]       mass of pendulum
params.g = 9.81; % [m/s^2]    acceleration of gravity
params.b = 0.01; % [s*Nm/rad] friction coefficient

params.u_max = 7;
params.u_min = -params.u_max;

params.I = params.m*params.l^2/3; 

% Assumed feedback gains, used to construct a CLF.
params.Kp=6;
params.Kd=5;

params.clf.rate = 3;
params.weight.slack = 100000;

x0 = [0.76; 0.05];

ip_sys = InvertedPendulum(params);

[xs, us, ts, extraout] = rollout_controller( ...
    x0, ip_sys, ip_sys, @ip_sys.ctrlClfQp, sim_t, 'verbose_level', 1);

figure;
title('Inverted Pendulum: CLF-QP States');
subplot(2, 1, 1);
plot(ts, 180 * xs(1, :)/pi);
xlabel("t (sec)"); ylabel("theta (deg)");

subplot(2, 1, 2);
plot(ts, 180 * xs(2, :)/pi);
xlabel("t (sec)"); ylabel("dtheta (deg/s)");

figure;
plot(ts, us); hold on;
plot(ts, params.u_max*ones(size(ts, 1), 1), 'k--');
plot(ts, params.u_min*ones(size(ts, 1), 1), 'k--');
xlabel("t (sec)"); ylabel("u (N.m)");