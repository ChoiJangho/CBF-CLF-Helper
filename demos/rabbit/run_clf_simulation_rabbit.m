%% RABBIT MODEL SIMULATION
%   WonsuhkJung smw04015@snu.ac.kr
% This is CBF-CLF Helper version of RABBIT Bipedal Walker.
% You can run the rabbit model using various controllers. You can check the
% available controllers in @CtrlAffineSysFL/ and @CtrlAffineSys/.
% This script is constitued of six sections. 
%   Section 1. Set simulation settings and model settings
%   Section 2. Define the rabbit system based on the params setting.
%   Section 3. Initialize the state to start simulation
%   Section 4. Run simulation using rollout_controller_for_multiple_resets
%   Section 5. Plot the result of the simulation
%   Section 6. Animate the trajectory of the RABBIT model.
%
% Model Setting
%   You can checkout init_clf_rabbit_simulation.m for model parameter
%   settings. Some of them are fragile, so you might not touch it to
%   reproduce the same results you've seen in paper.
%
% Simulation Setting
%   dt: control period
%   nstep: number of steps that RABBIT would take
%   with slack:
%       0: use slack variable while solving QP
%       1: not use slack variable while solving QP
%   verbose_level: 
%       0: print no log (default)
%       1: print log of each steps
%       2: print also the log of the rollout
%       3: print also the log of the controller
%
%
% Controller & Sensor
%   clf_qp_controller: Controller that defines the behavior in continuous
%                      time domain. You can define the controller you want
%                      to use here.
%   reset_map_func: Action-definer that decides the behavior at the moment of
%                   reset map. In this script, we just flip the role of
%                   stance leg and swing leg.
%   reset_event_func: Event-detector that decides the moment to reset map.
%                     In this script, it detects when RABBIT's left toe
%                     hits ground.

close all; clear all;
%% Step 1. Set Simulation Setting and System-parameters
% Tune the simulation setting and system parameters here.
% Simulation Setting
    % dt: control period of RABBIT model.
    % nstep: number of step that RABBIT would take
% params: all the system-related parameters
% verbose_level: 
%   0: print no log (default)
%   1: print log of each steps
%   2: print also the log of the rollout
%   3: print also the log of the controller
% with slack:
%   0: use slack variable while solving QP
%   1: not use slack variable while solving QP

init_clf_simulation_rabbit;
dt = 0.025;
nstep = 5;
with_slack = true;
verbose_level = 1;

%% Step 2. Prepare System
rabbit_sys = RabbitBuiltIn(params);

clf_qp_controller = @(t, x, varargin) rabbit_sys...
        .ctrlFeedbackLinearize(t, x, @rabbit_sys.ctrlClfQpFL, varargin{:});
reset_event_func = @rabbit_sys.rabbit_event;
reset_map_func = @rabbit_sys.reset_map;
exit_func = @rabbit_sys.exit_event;

%% Step 3. Initialize state
% Initialize the state vector.
x0 = [q0_ini;dq0_ini];
x = x0;
t0 = 0;

%% Step 4. Main Simulation
% Rollout the simulation
[xs, us, ts, extras] = rollout_controller_for_multiple_resets(...
    x0, rabbit_sys, clf_qp_controller, ...
    reset_event_func, reset_map_func, nstep,...
    'verbose_level', verbose_level, ...
    'dt', dt, 'T_exit', 1, 'exclude_pre_reset', 1, 'exit_function', exit_func);
xs = xs';
ts = ts';

%% Step 5. Plot the result
plot_rabbit_result(xs, ts, us, extras);

%% Step 6. Five Link Animation
% This is only designed for enabling animation of bipedal walker
global animation_scale 
animation_scale = rabbit_sys.params.scale;
animation_dt = 0.05;
x_quan_vec = coordinateTransformation(xs);
% anim_flat_ground(ts, x_quan_vec, animation_dt, true)
anim_flat_ground(ts', x_quan_vec', animation_dt, true)