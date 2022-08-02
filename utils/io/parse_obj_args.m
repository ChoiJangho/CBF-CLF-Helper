function obj = parse_obj_args(obj,varargin)
%% obj = parse_obj_args(obj, varargin{:});
    for idx = 1:2:length(varargin)
        obj.(varargin{idx}) = varargin{idx+1} ;
    end
end