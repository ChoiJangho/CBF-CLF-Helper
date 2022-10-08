clear all;
% close all;
dt = 0.01;

% Choice of Model Type: options:
%   'ONES': set every model parameters to 1 (including the gravity)
%   'QUANSER': Using the full quanser parameter values
%   'QUANSER_NO_DRAG': Using the full quanser parameter values except for
%   the drag terms.
%   'QUANSER_SIMPLE': Use only the mass and the length values from Quanser.
% Choice of Quanser weight type: 'NO_LOAD', 'WEIGHT', '2WEIGHTS'
params = get_predefined_params_set_for_vanilla_cart_pole('QUANSER', 'NO_LOAD');
% Set up input saturation limit.
params.u_max = (params.m + params. M) * 10;
% Set up CLF-related parameters
% params.clf.rate = 0.5;
params.weight_slack = 1e10;
% Create the dynamic system to simulate.
dynsys_force = CartPole(params);
dynsys_force.init_constraints(dynsys_force.params, 'init_cbf', false);

% Set up new input bound for the Quanser dynsys.
params_quanser = get_predefined_params_set_for_vanilla_cart_pole('QUANSER', 'NO_LOAD');
dynsys = QuanserCartPole(params_quanser, '2WEIGHTS');
plant = QuanserCartPole(params, '2WEIGHTS');

%% Define controllers to test out.
controller_sontag_up = @(t, x, varargin) dynsys_force.ctrl_clf_sontag(t, x, varargin{:});
controller_lqr_up = @(t, x, varargin) dynsys_force.ctrl_lqr_feedback(t, x, varargin{:});
controller_clf_qp_up = @(t, x, varargin) dynsys_force.ctrl_clf_qp(t, x, varargin{:});
controller_clf_min_norm_up = @(t, x, varargin) dynsys_force.ctrl_clf_min_norm(t, x, varargin{:});

controller_sontag_down = @(t, x, varargin) dynsys_force.ctrl_clf_sontag_to_stable_eq(t, x, varargin{:});
controller_lqr_down = @(t, x, varargin) dynsys_force.ctrl_lqr_feedback_to_stable_eq(t, x, varargin{:});

T = 5;
%% Testing simulation for the upright equilibrium.
controllers_upright = {controller_clf_min_norm_up, controller_sontag_up, controller_clf_qp_up, controller_lqr_up};
legends_upright = {'Min Norm', 'Sontag', 'CLF-QP', 'LQR'};
results_upright = cell(size(controllers_upright));
for i = 1:length(controllers_upright)
    x0 = [0; pi/96; 0; 0];

    %% Low-level controller maps desired force to input voltage.
    controller = @(t, x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        t, x, controllers_upright{i}, varargin{:});
    [xs, us, ts, extraout] = rollout_controller(x0, plant, controller, T);
%    [xs, us, ts, extraout] = rollout_controller(x0, dynsys, controllers_upright{i}, T);

    Vs = dynsys_force.clf(xs);
    result_i.stamps = ts;
    result_i.trajectory = xs;
    result_i.controls = us;
    result_i.legend = legends_upright{i};
    result_i.LineWidth = 2;
    results_upright{i} = result_i;
end

%% Testing simulation for the stable equilibrium.
controllers_down = {controller_sontag_down, controller_lqr_down};
legends_down = {'Sontag', 'LQR'};
results_down = cell(size(controllers_down));
for i = 1:length(controllers_down)
    x0 = [0; pi - pi/36; 0; 0];

    %% Low-level controller maps desired force to input voltage.
    controller = @(t, x, varargin) dynsys.ctrl_voltage_for_desired_force( ...
        t, x, controllers_down{i}, varargin{:});
    [xs, us, ts, extraout] = rollout_controller(x0, plant, controller, T);
%    [xs, us, ts, extraout] = rollout_controller(x0, dynsys, controllers_down{i}, T);

    result_i.stamps = ts;
    result_i.trajectory = xs;
    result_i.controls = us;
    result_i.legend = legends_down{i};
    result_i.LineWidth = 2;
    results_down{i} = result_i;
end


vis_state_history(results_upright,...
    'ylabel_descriptions', ["$p_x[m]$", "$\theta[rad]$", "$v[m/s]$", "$d\theta[rad/s]$"],...
    'title_descriptions', ["Position x", "Pendulum Angle", "Velocity", "Angular Velocity"],...
    'window_name', "Stabilization around the upright position - State History");
vis_control_history(results_upright, ...
    'title_descriptions', "Voltage",...
    'window_name', "Stabilization around the upright position - Control History");

vis_state_history(results_down,...
    'ylabel_descriptions', ["$p_x[m]$", "$\theta[rad]$", "$v[m/s]$", "$d\theta[rad/s]$"],...
    'title_descriptions', ["Position x", "Pendulum Angle", "Velocity", "Angular Velocity"],...
    'window_name', "Stabilization around the stable equilibrium - State History");
vis_control_history(results_down, ...
    'title_descriptions', "Voltage",...
    'window_name', "Stabilization around the stable equilibrium - Control History");

