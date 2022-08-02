function [xs, us, ts, extras] = rollout_controller_for_multiple_resets(...
    x0, plant_sys, control_sys, controller, ...
    reset_event_function, reset_map_function, n_reset, ...
    varargin)
%%   Rollout control for the hybrid system that uses reset_map functions
%%   Input:
%       Necessary
%           reset_event_function: function handle that detect the reset.
%           reset_map_function: function handle that defines the reset map.
%           n_reset: number of reset to simulate.
%       Default
%           varargin: with_slack, mu0, verbose, event_options
%%   Supported fields for varargin
%       T_exit: exit simulation if one rollout exit this time before
%       hitting the reset map.
%       exclude_pre_reset: excludes pre reset xs, us, ts, etc from the
%       traces. (default: 0)
%       exit_function: if this exit condition is satisfied, terminate
%       simulation without going further.
%   DEBUG:
%   verbose_level: 
%       0: print no log (default)
%       1: print log of each steps
%       2: print also the log of the rollout
%       3: print also the log of the controller
%   if 'verbose' is set to 1(0), it sets verbose_level = 1(0)
%%   Output:
%       xs: trajectory
%       us: control inputs
%       ts: time stamps

settings = parse_function_args(varargin{:});

if ~isfield(settings, 't0')
    t0 = 0;
else
    t0 = settings.t0;
end

if ~isfield(settings, 'T_exit')
    fprintf("Warning: T_exit is set to default value of 1e4. It is strongly %s", ...
        "recommended to manually set T_exit to prevent unnecessary rollout.\n");
    T_exit = 1e4;
else
    T_exit = settings.T_exit;
end

if ~isfield(settings, 'exclude_pre_reset')
    exclude_pre_reset = 0;
else
    exclude_pre_reset = settings.exclude_pre_reset;
end

if ~isfield(settings, 'exit_function')
    exit_function = [];
else
    exit_function = settings.exit_function;
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

verbose_level_rollout = max(verbose_level-1, 0);

x = x0;
% initialize traces.
xs = [];
ts = [];
us = [];
index_reset = [];
extras = [];

%% Run Step Simulation
for k = 1:n_reset
    step_summary.x_init = x0;
    [xs_new, us_new, ts_new, extras_new] = rollout_controller( ...
        x0, plant_sys, control_sys, controller, T_exit,...
        varargin{:}, 'verbose_level', verbose_level_rollout, 't0', t0, ...
        'end_event_function', reset_event_function);
    end_with_reset = extras_new.end_with_event;
    extras_new = rmfield(extras_new, 'end_with_event');
    step_summary.x_terminal = xs_new(:, end);
    step_summary.T = ts_new(end) - ts_new(1);
    step_summary.end_with_event = end_with_reset;
    if verbose_level >= 1
        print_step_summary(k, step_summary);
    end
    % Save to log.
    %% TODO: this can be changed based on the exit_function.
    [exit_flag, exit_valid_index] = exit_function(xs_new);
    if exclude_pre_reset
        margin = 1;
    else
        margin = 0;
    end
    
    if exit_flag
        % normally exited
        index_last = exit_valid_index - margin;
    else
        index_last = length(ts_new) - margin;
    end
    
    extras = fetch_extras(extras, extras_new, index_last);   
    ts = [ts, ts_new(1:index_last)];
    xs = [xs, xs_new(:, 1:index_last)];
    us = [us, us_new(:, 1:index_last)];

    if ~end_with_reset || exit_flag
        % One step rollout terminated without hitting the reset. Terminate
        % the simulation here.
        break;
    else
        index_reset = [index_reset; length(ts)];
    end
    %% reset to next initial state.
    x0 = reset_map_function(xs_new(:, end)');
    t0 = ts_new(end);    
end
extras.index_reset = index_reset;
end % end of the main function.

function extras = fetch_extras(extras, extras_new, index_last)
    if isempty(extras)
        extras_field_name = fieldnames(extras_new);
    else
        extras_field_name = fieldnames(extras);
    end
    
    n_field = length(extras_field_name);
    for i_field = 1:n_field
        to_attach = extras_new.(extras_field_name{i_field});
        if iscell(to_attach)
            to_attach = to_attach(1:index_last);
            if isfield(extras, extras_field_name{i_field})
                extras.(extras_field_name{i_field})(end+1:end+index_last) = ...
                    to_attach;
            else
                extras.(extras_field_name{i_field}) = to_attach;
            end
        else
            otherdims = repmat({':'},1,ndims(to_attach)-1);
            to_attach = to_attach(otherdims{:}, 1:index_last);
            if isfield(extras, extras_field_name{i_field})
                extras.(extras_field_name{i_field}) = ...
                    cat(ndims(to_attach), extras.(extras_field_name{i_field}), to_attach);
            else
                extras.(extras_field_name{i_field}) = to_attach;
            end            
        end

    end
end

function print_step_summary(i, log)
    fprintf("------Summary of %d-th step------\n", i);
    disp("Initial state");
    disp(log.x_init');
    disp("Terminal state");
    disp(log.x_terminal');
    fprintf("Time spent: %.3f \t end_with_event: %d \n", [log.T, log.end_with_event]);
    fprintf("------Resetting to next step------\n\n");
end
