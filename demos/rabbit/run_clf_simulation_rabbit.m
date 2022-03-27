%% RABBIT MODEL SIMULATION
%   WonsuhkJung smw04015@snu.ac.kr
% This is CBF-CLF Helper version of RABBIT Bipedal Walker.
% You can run the rabbit model using various controllers. You can check the
% available controllers in @CtrlAffineSysFL/ and @CtrlAffineSys/.
% This script is constitued of six sections. 
%   Section 1. Set simulation settings and model settings
%   Section 2. Define the control system, plant system and reflect model
%           uncertianty to the model.
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
% Uncertainty Setting
%   You can reflect the uncertainty of the model by tuning this value.
%   plant_sys.params.scale = mass-scale ratio
%   plant_sys.params.torso_add = additional torso mass
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
% dt = 0.002;
nstep = 10;
with_slack = true;
verbose_level = 1;

%% Step 2. Prepare System (Uncertainty control)
control_sys = RabbitBuiltIn(params);
plant_sys = RabbitBuiltIn(params);

% Reflect model uncertainty here
plant_sys.params.scale = 1.2;
%plant_sys.params.torso_add = 10;

clf_qp_controller = @(x, varargin) control_sys...
        .ctrlFeedbackLinearize(x, @control_sys.ctrlClfQpFL, varargin{:});
reset_event_func = @plant_sys.rabbit_event;
reset_map_func = @plant_sys.reset_map;
exit_func = @control_sys.exit_event;

%% Step 3. Initialize state
% Initialize the state vector.
x0 = [q0_ini;dq0_ini];
x = x0;
t0 = 0;

%% Step 4. Main Simulation
% Rollout the simulation
[xs, us, ts, extras] = rollout_controller_for_multiple_resets(...
    x0, plant_sys, control_sys, clf_qp_controller, ...
    reset_event_func, reset_map_func, nstep,...
    'with_slack', with_slack, 'verbose_level', verbose_level, ...
    'dt', dt, 'T_exit', 1, 'exclude_pre_reset', 1, 'exit_function', exit_func);
xs = xs';
ts = ts';

%% Step *. Experiment
close all;
for scale = 1.0:0.1:2.0
    LfV_mismatch_array = [];
    LgV_mismatch_array = [];
    plant_sys = RabbitBuiltIn(params);
    plant_sys.params.scale = scale;
    for idx = 1:size(simul_mat, 1)
        x_test = simul_mat(idx, :);
        [y, dy, L2fy, LgLfy, ~] = control_sys.eval_y(x_test');
        [y_hat, dy_hat, L2fy_hat, LgLfy_hat, ~] = plant_sys.eval_y(x_test');

        eta = [y; dy];
        delta_1 = L2fy_hat - LgLfy_hat * inv(LgLfy) * L2fy;
        delta_2 = LgLfy_hat * inv(LgLfy) - eye(4);
        P = control_sys.Gram_clf_FL;
        G = control_sys.G_FL;

        LfV_mismatch = 2*eta'*P*G*delta_1;
        LgV_mismatch = 2*eta'*P*G*delta_2;

        common = 2*eta'*P*G;
        common_norm = norm(common);

    %     LfV_mismatch = LfV_mismatch./common_norm;
    %     LgV_mismatch = LgV_mismatch./common_norm;

        LfV_mismatch_array = [LfV_mismatch_array; LfV_mismatch];
        LgV_mismatch_array = [LgV_mismatch_array; LgV_mismatch];
    end
    figure()
    subplot(511)
    plot(perturb_head, LfV_mismatch_array);
    title_descr = strcat("Mismatch term in time simulation", num2str(scale));
    title(title_descr);
    x_descr = strcat("LfV");
    ylabel(x_descr);
    grid on;
    for i = 1:4
        subplot(5,1,i+1)
        plot(perturb_head, LgV_mismatch_array(:,i));
        y_descr = strcat("LgV", num2str(i));
        ylabel(y_descr);
        grid on;
    end
end

%% Sensitivity test
% (1) scale sensitivity => graph style didn't change much.
% (2) eta sensitivity
% idea: we should fix other dimension of eta, while accessing to L2fy
%       freely. Is there a map from 
num_axis = 12;
axis = 9;
vec = zeros(num_axis, 1);
vec(axis) = 1;
perturb_dx = 0.002;
num_step = 50;
x_temp = xs(5, :);
perturb_head = x_temp(axis) : perturb_dx : x_temp(axis) + perturb_dx * (num_step-1);
simul_mat = repmat(x_temp, num_step, 1);
simul_mat(:, axis) = perturb_head;

%% Step 5. Plot the result
plot_rabbit_result(xs, ts, us, extras);

%% Step 6. Five Link Animation
% This is only designed for enabling animation of bipedal walker
% global animation_scale 
% animation_scale = plant_sys.params.scale;
% animation_dt = 0.05;
% x_quan_vec = coordinateTransformation(xs);
% anim_flat_ground(ts, x_quan_vec, animation_dt)