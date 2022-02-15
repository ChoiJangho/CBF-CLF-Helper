function [xs, us, ts, extraout] = rollout_controller( ...
    x0, plant_sys, control_sys, controller, T, varargin)
%% [xs, us, ts, extras] = rollout_controller( ...
%%    x0, plant_sys, control_sys, controller, T, varargin)
%% [xs, us, ts, extras] = rollout_controller( ...
%%    x0, plant_sys, control_sys, controller, T, ...
%%    'field1_name', field1_value, 'field2_name', field2_value)
% example:
% [xs, us, ts, extraout] = rollout_controller( ...
%    x0, t0, plant_sys, control_sys, @control_sys.ctrlClfQp, T);
% Vs = extraout.Vs;
% In this function, the controller always takes 'with_slack', 'verbose' as
% additional input arguments. Make sure your own controller supports these
% fields with varargin. If you want to pass other varying input arguments
% to the controller, follow the below sample code:
% controller_handle = @(x, varargin) your_custom_controller(x, ...
%    'field1_name', field1_value, 'field2_name', field2_value, varargin{:});
% [xs, us, ts, extraout] = rollout_controller( ...
%    x0, t0, plant_sys, control_sys, controller_handle, T);
%% simulates plant_sys with ctrlClfQp controller of the control_sys
%% for [t0, t0+T] starting at the initial state x0.
%% INPUTS
%   x0: initial state
%   plant_sys: ControlAffineSys instance for the plant simulation.
%   control_sys: ControlAffineSys instance for the model and the controller.
%   controller: function handle for controller.
%       examples: ctrlClfQp, ctrlCbfQp, ctrlCbfClfQp
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
%   with_slack: if 1: relax constraints, 0: hard constraints. (default 1)
%   u_ref: reference signal of u. It can be a constant vector or a function
%   handle that takes x as input.
%   t_tolerance: time tolerance for ending the simulation (default 1e-10)
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

% Feedback-linearizer Determinant
if isa(control_sys, 'CtrlAffineSysFL')
    is_control_sys_FL = true;
else
    is_control_sys_FL = false;
end

if ~isfield(settings, 't0')
    t0 = 0;
else
    t0 = settings.t0;
end

% TODO: reverse calculate mu0 from u0
if ~isfield(settings, 'u0')
    u0 = [];
else
    if length(settings.u0) ~= control_sys.udim
        error("u0 dimension does not match with control_sys.udim.");
    end    
    u0 = settings.u0;
end

if ~isfield(settings, 'u_ref')
    u_ref = [];
else
    if ~isa(settings.u_ref, 'function_handle') && ...
            (length(settings.u_ref) ~= control_sys.udim)
        error("u_ref dimension does not match with control_sys.udim.");        
    end
    u_ref = settings.u_ref;
end

if ~isfield(settings, 'mu0')
    mu0 = [];
else
    if ~is_control_sys_FL
        error("mu0 is supported only for control_sys of CtrlAffineSysFL");
    end
    if length(settings.mu0) ~= control_sys.udim
        error("mu0 dimension does not match with control_sys.udim.");
    end    
    mu0 = settings.mu0;
end

if ~isfield(settings, 'mu_ref')
    mu_ref = [];
else
    if ~is_control_sys_FL
        error("mu_ref is supported only for control_sys of CtrlAffineSysFL");
    end
    if length(settings.mu_ref) ~= control_sys.udim
        error("mu_ref dimension does not match with control_sys.udim.");        
    end
    mu_ref = settings.mu_ref;
end

if ~isempty(u0) && ~isempty(mu0)
    error("Only one of u0 and mu0 should be provided.");
end

if isempty(u_ref) && isempty(mu_ref)
    ref_type = 0;
elseif ~isempty(u_ref) && ~isempty(mu_ref)
    error("Only one of u_ref and mu_ref should be provided.");
elseif ~isempty(u_ref)
    ref_type = 1; % reference signal is u.
else
    ref_type = 2; % reference signal is mu.
end

if ~isfield(settings, 'dt')
    dt = 0.01;
else
    dt = settings.dt;
end

if ~isfield(settings, 'with_slack')
    with_slack = 1;
else
    with_slack = settings.with_slack;
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

% Dimension check
if plant_sys.udim ~= control_sys.udim
    error("plant_sys.udim and control_sys.udim should match each other.")
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
u_prev = zeros(control_sys.udim, 1);
mu_prev = zeros(control_sys.udim, 1);

end_simulation = false;
MAX_TIME = 20;
%% Run simulation.
% _t indicates variables for the current loop.
tstart = tic;
while ~end_simulation
    %% Determine control input.
    tstart = tic; % DEBUG
    
    if t == t0 && ~isempty(u0)
        u = u0;
        % Run dummy controller to get extra_t
        % TODO: this is a bad practice, fix this.
        
        [~, extra_t] = controller(x, 'verbose', 0);
        
        extra_t.feas = 1;
        extra_t.comp_time = 0;
    elseif t == t0 && ~isempty(mu0)
        mu = mu0;
        u = control_sys.ctrlFeedbackLinearize(x, mu);
        % Run dummy controller to get extra_t
        % TODO: this is a bad practice, fix this.
        [~, extra_t] = controller(x, 'verbose', 0);
        extra_t.feas = 1;
        extra_t.comp_time = 0;
    else
        if ref_type == 0
            [u, extra_t] = controller(x, ...
                'with_slack', with_slack, 'verbose', (verbose_level>=2));            
            
        elseif ref_type == 1
            if isa(u_ref, 'function_handle')
                u_ref_t = u_ref(x, u_prev);
            else
                u_ref_t = u_ref;
            end
            [u, extra_t] = controller(x, 'u_ref', u_ref_t, ...
                'with_slack', with_slack, 'verbose', (verbose_level>=2));                        
        elseif ref_type == 2
            if isa(mu_ref, 'function_handle')
                mu_ref_t = mu_ref(x, mu_prev);
            else
                mu_ref_t = mu_ref;
            end            
            [u, extra_t] = controller(x, 'mu_ref', mu_ref_t, ...
                'with_slack', with_slack, 'verbose', (verbose_level>=2));                        
        end
    end
    if verbose_level >= 1
        print_log(t, x, u, extra_t);
    end
    
    tend = toc(tstart);
    
    us = [us, u];
    extraout = fetch_other_extras(extraout, extra_t);
    feas = [feas, extra_t.feas];
    if ~isfield(extra_t, 'comp_time')
        comp_times = [comp_times, 0];
    else
        comp_times = [comp_times, extra_t.comp_time];        
    end
    if with_slack
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
        % TODO: When ODE Fails due to divergence, infinite-loop
        t_ode = toc(tstart);
        if t_ode > MAX_TIME
            break
        end
        end_simulation = abs(ts_t(end) - (t0 + T))<t_tolerance;
        end_with_event = [];
    end            
    t = ts_t(end);
    x = xs_t(end, :)';
    u_prev = u;
    %% Record traces.
    xs = [xs, x];
    ts = [ts, t];
end % end of the main while loop
%% Add control input for the final timestep.
if ref_type == 0
    [u, extra_t] = controller(x, ...
        'with_slack', with_slack, 'verbose', (verbose_level>=2));            
elseif ref_type == 1
    if isa(u_ref, 'function_handle')
        u_ref_t = u_ref(x, u_prev);
    else
        u_ref_t = u_ref;
    end
    [u, extra_t] = controller(x, 'u_ref', u_ref_t, ...
        'with_slack', with_slack, 'verbose', (verbose_level>=2));                        
elseif ref_type == 2
    if isa(mu_ref, 'function_handle')
        mu_ref_t = mu_ref(x, mu_prev);
    else
        mu_ref_t = mu_ref;
    end            
    [u, extra_t] = controller(x, 'mu_ref', mu_ref_t, ...
        'with_slack', with_slack, 'verbose', (verbose_level>=2));                        
end
if verbose_level >= 1
    print_log(t, x, u, extra_t);
end


us = [us, u];
extraout = fetch_other_extras(extraout, extra_t);
feas = [feas, extra_t.feas];
comp_times = [comp_times, extra_t.comp_time];
if with_slack
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
    if length(fieldnames(extras)) == 0
        extras_field_name = fieldnames(extra_t);
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
        fprintf("\t feas: %d", extra_t.feas);
        fprintf("\t comp_time: %.2f", extra_t.comp_time);
        % Add custom log here.
        fprintf("\n");
end
