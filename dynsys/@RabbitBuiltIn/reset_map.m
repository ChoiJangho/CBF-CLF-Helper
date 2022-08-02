function x_after = reset_map(obj, x_before, varargin)
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
end