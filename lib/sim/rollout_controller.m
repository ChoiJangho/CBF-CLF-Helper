function [xs, us, ts, extraout] = rollout_controller( ...
    x0, plant_sys, controller, T, varargin)
% | ``[xs, us, ts, extras] = rollout_controller(x0, plant_sys, controller, T, varargin)``
% | ``[xs, us, ts, extras] = rollout_controller(x0, plant_sys, controller, T, ...``
% |   ``'field1_name', field1_value, 'field2_name', field2_value)``
% Examples:
%  simulates plant_sys with ctrl_clf_qp controller of the control_sys for
%  [t0, t0+T] starting at the initial state x0::
% 
%    [xs, us, ts, extraout] = rollout_controller(x0, t0, plant_sys, @control_sys.ctrlClfQp, T);
%    Vs = extraout.Vs;
% 
% In this function, the controller always takes 'with_slack', 'verbose' as
% additional input arguments. Make sure your own controller supports these
% fields with varargin. If you want to pass other varying input arguments
% to the controller, follow the below sample code::
% 
%  controller_handle = @(t, x, varargin) your_custom_controller(t, x, ...
%    'field1_name', field1_value, 'field2_name', field2_value, varargin{:});
%  [xs, us, ts, extraout] = rollout_controller( ...
%    x0, t0, plant_sys, controller_handle, T);
% 
%% INPUTS
%   x0: initial state
%   plant_sys: ControlAffineSys instance for the plant simulation.
%   controller: function handle for controller.
%       examples: ctrl_clf_qp, ctrl_cbf_qp, ctrl_cbf_clf_qp
%   T: simulation time (time horizon)
%% Supported fields for varargin
%   SIMULATION OPTIONS:
%   t0: initial t (default: 0.0)
%   u0: initial control input (if you want to specify the first input.)
%   dt: sampling time (default: 0.01)
%   end_event_function: function handle that ends the simulation, for more
%   information, please check out matlab ode Events.
%   ode_func: your own choice of ode_func for simulation. (default: ode45)
%   CONTROLLER OPTIONS:
%   t_tolerance: time tolerance for ending the simulation (default 1e-10)
%   
%   DEBUG:
%   verbose_level: 
%       0: print no log (default)
%       1: print log of the rollout
%       2: print also the log of the controller
%   if 'verbose' is set to 1(0), it sets verbose_level = 1(0)
%% OUTPUTS
%   xs: trace of state (size: (plant_sys.xdim, total_k))
%   us: trace of control input (size: (control_sys.udim, total_k))
%       Note that the last control input in the array is actually not used
%       in the simulation. It is appended to match the size with xs and ts.
%   ts: trace of time (length: total_k)
%% Extra outputs (extraout)
%   Vs: trace of the CLF values (size: (control_sys.n_clf, total_k))
%   Bs: trace of the CLF values (size: (control_sys.n_cbf, total_k))
%   feas: trace of feasibility of the optimization-based controller (length: total_k)
%   slacks: trace of slack variables (size: (n_slack, total_k))
%       Note that n_slack depends on the choice of the controller.
%   comp_times: trace of computation time for each timestep (length: total_k)
%   ys: trace of the output defined for feedback linearization (size: (control_sys.ydim, total_k))
%   mus: trace of virtula control input for feedback linearizationb-based
%       controller (size: (control_sys.udim, total_k))
%   end_with_event: 1 if the simulation ended with the end_event, else 0.
settings = parse_function_args(varargin{:});

if ~isfield(settings, 't0')
    t0 = 0;
else
    t0 = settings.t0;
end

if ~isfield(settings, 'u0')
    u0 = [];
else
    if length(settings.u0) ~= plant_sys.udim
        error("u0 dimension does not match with plant_sys.udim.");
    end    
    u0 = settings.u0;
end

if ~isfield(settings, 'dt')
    dt = 0.01;
else
    dt = settings.dt;
end

if ~isfield(settings, 'verbose_level')
    if isfield(settings, 'verbose')
        verbose_level = settings.verbose;
    else
        verbose_level = 0;
    end
else
    verbose_level = settings.verbose_level;
end

if ~isfield(settings, 'end_event_function')
    end_event_function = [];
else
    end_event_function = settings.end_event_function;
end

if ~isfield(settings, 'ode_func')
    ode_func = @ode45;
else
    ode_func = settings.ode_func;
end

if ~isfield(settings, 't_tolerance')
    t_tolerance = 1e-10;
else
    t_tolerance = settings.t_tolerance;
end

udim = plant_sys.udim;
if length(x0) ~= plant_sys.xdim
    error("x0 dimension does not match with plant_sys.xdim.");
end
if isrow(x0)
    x0 = x0';
end

% Initialize traces.
xs = x0;
ts = t0;
us = [];
% traces for extras, they remain empty if not returned.
feas = [];
comp_times = [];
slacks = [];
Vs = [];
Bs = [];
% traces for extras, specific to feedback linearize-based controller.
ys = [];
dys = [];
mus = [];
% traces for other extras
extraout = struct;

% Initialize state & time.
x = x0;
t = t0;
u_prev = zeros(plant_sys.udim, 1);

end_simulation = false;
%% Run simulation.
% _t indicates variables for the current loop.
tstart = tic;

% Run dummy controller to get necessary arguments in extra outputs and
% check control input dimension is correct.
[u_dummy, extra_dummy] = controller(t, x, 'verbose', 0);
if length(u_dummy) ~= udim
    error("controller output u size is different from plant_sys.udim");
end

if isfield(extra_dummy, 'u_prev')
    pass_u_prev = true;
else
    pass_u_prev = false;
end

while ~end_simulation
    tstart = tic;
    
    %% Determine control input.
    if t == t0 && ~isempty(u0)
        u = u0;
        extra_t = extra_dummy;
    elseif pass_u_prev
        [u, extra_t] = controller(t, x, ...
            'u_prev', u_prev, 'verbose', (verbose_level>=2));
    else
        [u, extra_t] = controller(t, x, 'verbose', (verbose_level>=2));
    end
    if verbose_level >= 1
        print_log(t, x, u, extra_t);
    end
        
    us = [us, u];
    extraout = fetch_other_extras(extraout, extra_t);
    if isfield(extra_t, 'feas')
        feas = [feas, extra_t.feas];
    end
    if ~isfield(extra_t, 'comp_time')
        comp_times = [comp_times, -1];
    else
        comp_times = [comp_times, extra_t.comp_time];        
    end
    if isfield(extra_t, 'slack')        
        slacks = [slacks, extra_t.slack];
    end
    if isfield(extra_t, 'Vs')
        Vs = [Vs, extra_t.Vs];
    end
    if isfield(extra_t, 'Bs')
        Bs = [Bs, extra_t.Bs];
    end
    if isfield(extra_t, 'y')
        ys = [ys, extra_t.y];
    end
    if isfield(extra_t, 'dy')
        dys = [dys, extra_t.dy];
    end
    if isfield(extra_t, 'mu')
        mus = [mus, extra_t.mu];
        mu_prev = extra_t.mu;
    end        
    
    %% Run simulation for one time step.
    t_end_t = min(t + dt, t0+T);
    if ~isempty(end_event_function)
        ode_opt = odeset('Events', end_event_function);
        [ts_t, xs_t, t_event] = ode_func( ...
            @(t, x) plant_sys.dynamics(t, x, u), ...
            [t, t_end_t], x, ode_opt);
        end_simulation = abs(ts_t(end) - (t0 + T))<t_tolerance || ~isempty(t_event);
        end_with_event = ~isempty(t_event);
    else
        [ts_t, xs_t] = ode_func(@(t, x) plant_sys.dynamics(t, x, u), ...
            [t, t_end_t], x);
        end_simulation = abs(ts_t(end) - (t0 + T))<t_tolerance;
        end_with_event = [];
    end            
    t = ts_t(end);
    x = xs_t(end, :)';
    x = plant_sys.clip_state_angles(x);
    u_prev = u;
    %% Record traces.
    xs = [xs, x];
    ts = [ts, t];
end % end of the main while loop
%% Add control input for the final timestep.
if pass_u_prev
    [u, extra_t] = controller(t, x, ...
        'u_prev', u_prev, 'verbose', (verbose_level>=2));
else        
    [u, extra_t] = controller(t, x, 'verbose', (verbose_level>=2));
end
if verbose_level >= 1
    print_log(t, x, u, extra_t);
end

us = [us, u];
extraout = fetch_other_extras(extraout, extra_t);
if isfield(extra_t, 'feas')
    feas = [feas, extra_t.feas];
end
if ~isfield(extra_t, 'comp_time')
    comp_times = [comp_times, -1];
else
    comp_times = [comp_times, extra_t.comp_time];        
end
if isfield(extra_t, 'slack')
    slacks = [slacks, extra_t.slack];
end
if isfield(extra_t, 'Vs')
    Vs = [Vs, extra_t.Vs];
end
if isfield(extra_t, 'Bs')
    Bs = [Bs, extra_t.Bs];
end
if isfield(extra_t, 'y')
    ys = [ys, extra_t.y];
end
if isfield(extra_t, 'dy')
    dys = [dys, extra_t.dy];
end
if isfield(extra_t, 'mu')
    mus = [mus, extra_t.mu];
end

%% Make extraout
extraout.feas = feas;
extraout.comp_times = comp_times;
if ~isempty(slacks)
    extraout.slacks = slacks;
end
if ~isempty(Vs)
    extraout.Vs = Vs;
end
if ~isempty(Bs)
    extraout.Bs = Bs;
end
if ~isempty(ys)
    extraout.ys = ys;
end
if ~isempty(dys)
    extraout.dys = dys;
end
if ~isempty(mus)
    extraout.mus = mus;
end
if ~isempty(end_with_event)
    extraout.end_with_event = end_with_event;
end
end % end of the main function.


function extras = fetch_other_extras(extras, extra_t)
    if isempty(extras)
        if ~isempty(extra_t)
            extras_field_name = fieldnames(extra_t);
        else
            extras_field_name = [];
        end
    else
        extras_field_name = fieldnames(extras);
    end
    
    n_field = length(extras_field_name);
    for i_field = 1:n_field
        if ~strcmp(extras_field_name{i_field}, 'feas') && ...
                ~strcmp(extras_field_name{i_field}, 'comp_time') && ...
                ~strcmp(extras_field_name{i_field}, 'slack') && ...
                ~strcmp(extras_field_name{i_field}, 'Vs') && ...
                ~strcmp(extras_field_name{i_field}, 'Bs') && ...
                ~strcmp(extras_field_name{i_field}, 'y') && ...
                ~strcmp(extras_field_name{i_field}, 'mu') && ...
                ~strcmp(extras_field_name{i_field}, 'dy')
            to_attach = extra_t.(extras_field_name{i_field});
            if ~isfield(extras, extras_field_name{i_field})
                extras.(extras_field_name{i_field}) = {};
            end
            extras.(extras_field_name{i_field}){end+1} = to_attach;
        end
    end
end

function print_log(t, x, u, extra_t)
        fprintf("t: %.3f, \t x: ", t);
        fprintf("%.2g, ", x);
        fprintf("\t u: ");
        fprintf("%.2g, ", u);
        if isfield(extra_t, 'feas')
            fprintf("\t feas: %d", extra_t.feas);
        end
        if isfield(extra_t, 'comp_time')
            fprintf("\t comp_time: %.2f", extra_t.comp_time);
        end
        % Add custom log here.
        fprintf("\n");
end
