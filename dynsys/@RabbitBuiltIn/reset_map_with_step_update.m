function x_after = reset_map_with_step_update(obj, x_before, varargin)
%% Reset Map Function
% varargin should have the environment setting
    
    % TODO: generalize this to environment setting class.
    environment = parse_function_args(varargin{:});
    
    if ~isfield(environment, 'perturbation')
        perturbation = 0;
    else
        perturbation = environment.perturbation;
    end

    n = obj.xdim/2;
    q = obj.reset_map_matrix * x_before(1:n)';
    dq = obj.reset_map_matrix * obj.dq_pos_gen_v2([x_before(1:n) x_before(n+1:2*n)]);
    x_after = [q; dq];
    if perturbation ~= 0
        x_after = obj.perturb(x_after, perturbation); % not neat though.
    end
    
    if ~isempty(obj.steps_min)
        if obj.step_index < obj.steps_min
            obj.step_index = obj.step_index + 1;
            obj.step_min = obj.steps_min(obj.step_index);
            obj.step_max = obj.steps_max(obj.step_index);
        end
    end
end