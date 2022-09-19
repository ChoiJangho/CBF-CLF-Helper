function [u, extraout] = ctrl_clip_input(obj, t, x, varargin)

    kwargs = parse_function_args(varargin{:});
    if ~isfield(kwargs, 'u_ref')
        error("u_ref must be given as extra input argument.");
    else
        u_ref = kwargs.u_ref;
        if isa(u_ref, 'function_handle')
            varargin = obj.remove_latest_u_ref(varargin);            
            [u_ref_, extraout] = u_ref(t, x, varargin{:});
            extraout = obj.append_u_ref(extraout, u_ref_);
        elseif isa(u_ref, 'numeric')
            u_ref_ = u_ref;
            extraout.u_ref = u_ref_;
        end
    end
    
    if ~isfield(kwargs, 'u_max')
        u_max = obj.u_max;
    else
        u_max = kwargs.u_max;
    end
    
    if ~isfield(kwargs, 'u_min')
        u_min = obj.u_min;
    else
        u_min = kwargs.u_min;
    end
    
    u = obj.clip_input(u_ref_, u_max, u_min);
end