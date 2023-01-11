function init_sys_FL(obj)
    if obj.is_fl_initialized
        return
    end
%% Functions that initialize dynamic system
    %% Symbolic Function
    if strcmp(obj.setup_option, 'symbolic')
        disp(['Setting up feedback linearization dynamics, CLFs, CBFs from defined symbolic expressions.', ...
            '(This might take time.)']);
        [x, f, g] = obj.defineSystem(obj.params);
        if strcmp(obj.output_option, 'siso')
            [y, z] = obj.defineOutputWithZeroCoords(obj.params, x);
            obj.initOutputDynamicsSiso(x, f, g, y, z);
        elseif strcmp(obj.output_option, 'mimo')
            [y, z] = obj.defineOutputWithZeroCoords(obj.params, x);
            obj.initOutputDynamicsMimo(x, f, g, y, z);            
        elseif strcmp(obj.output_option, 'phase')
            [y, phase, y_max_exceed, y_min_exceed] = obj.defineOutputWithPhase(obj.params, x);
            obj.initOutputDynamics(x, f, g, y, phase, y_max_exceed, y_min_exceed, obj.params);            
        else
            error("Unknown output_option.");
        end
    
    %% Non Symbolic Function
    elseif strcmp(obj.setup_option, 'built-in')
        if ~isfield(obj.params, 'rel_deg_y')
            error("rel_deg_y should be specified for built-in setup.");
        end
        obj.rel_deg_y = obj.params.rel_deg_y; % TODO: other way to judge it?
        if obj.rel_deg_y ~= 2
            error("Not Suppported");
        end
        obj.ydim = obj.udim;
    else
        error("Undefined setup_option.");
    end
    %% Set up desired linear output dynamics
    %% Both Symbolic and Non-symbolic
    if ~isfield(obj.params, 'epsilon_FL') && ~isfield(obj.params, 'eps_FL')
        disp(['[Warning] The rate of RES-CLF, epsilon_FL, is set to a default value 1', ...
            'It is strongly recommended to use a custom value (which can be specified by obj.params.epsilon_FL).']);
        eps = 1;
    elseif isfield(obj.params, 'epsilon_FL')
        eps = obj.params.epsilon_FL;
        obj.params = rmfield(obj.params, 'epsilon_FL');
    elseif isfield(obj.params, 'eps_FL')
        eps = obj.params.eps_FL;
        obj.params = rmfield(obj.params, 'eps_FL');
    end
    obj.eps_FL = eps;
    
    if strcmp(obj.output_option, 'siso')
        if obj.rel_deg_y == 1
            obj.F_FL = 0;
            obj.G_Fl = 1;
        else
            % Higher relative degree
            obj.F_FL = [zeros(obj.rel_deg_y-1, 1), eye(obj.rel_deg_y-1);
                        zeros(1, obj.rel_deg_y)];
            obj.G_FL = [zeros(obj.rel_deg_y-1, 1); 1];
        end
    elseif strcmp(obj.output_option, 'mimo')
        disp("Warning: Not implemented");
    elseif strcmp(obj.output_option, 'phase')
        obj.F_FL = [zeros(obj.ydim), eye(obj.ydim);
            zeros(obj.ydim), zeros(obj.ydim)];
        obj.F_FL_eps = [zeros(obj.ydim), 1/eps * eye(obj.ydim);
            zeros(obj.ydim), zeros(obj.ydim)];
        obj.G_FL = [zeros(obj.ydim);
                    eye(obj.ydim)];
    else
        error("NotImplemented");
    end
    
    %% TODO: temporal code structure. Discuss with Wonsuhk how to deal with this.
    if strcmp(obj.output_option, 'phase')
        obj.initClfFl(obj.params);
    end
    
    obj.is_fl_initialized = true;
end