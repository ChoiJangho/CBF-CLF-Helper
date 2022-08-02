clear all;
close all;
dt = 0.01;

%% Direct CLF without FL works for these parameters.
% params.l = 1.0;    % [m]        length of pendulum
% params.m = 1.0;    % [kg]       mass of pendulum
% params.M = 1.0;    % [kg]       mass of cart
% params.g = 9.81;   % [m/s^2]    acceleration of gravity
% params.b = 0.1;    % [s*Nm/rad] friction coefficient

params.l = 0.5;    % [m]        length of pendulum
params.m = 0.5;    % [kg]       mass of pendulum
params.M = 2.0;    % [kg]       mass of cart
params.g = 9.81;   % [m/s^2]    acceleration of gravity
params.b = 0.1;    % [s*Nm/rad] friction coefficient

params.cbf.rate = 1;

u_bound = 50;
params.u_max = u_bound;
params.u_min = -u_bound;

params.ratio_u_diff = 0;
params.weight.slack = 1000;
%params.weight.input = eye(params.udim); % tune this to allocate different
%weight in each u coordinate
params.clf.rate = 1;

%% Feedback linearization specific params.
params.bezier = [-0.0630040307234950, ...
    -2.94395202983077, ...
    4.23655180750478, ...
    -10.8954633332227, ...
    13.3619387476085, ...
    -13.4630778220299, ...
    9.93449615595792, ...
    -3.07150993759866, ...
    3.86268687914481, ...
    2.72221821558065, ...
    3.15006153603993];
params.phase_max = 1;
params.phase_min = 0;
params.epsilon_FL = 0.015;
params.Kp_FL = 0.99;
params.Kd_FL = 2.0;

control_sys = CartPole(params);

plant_sys = CartPole(params);

settings.dt = dt;
